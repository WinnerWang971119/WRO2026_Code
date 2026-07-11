"""Cross-language golden-vector parity for the chain speed planner (T5: AC6 / TC1).

``tools.path_planner.ctps.chain_speed`` is the single Python source of the
two-pass collinear-run velocity planner the CTPS player and the offline sim
render (ADR-059, the 2026-07-10 collinear-run continuous-motion plan). It is
pinned to the firmware ``ChainSpeedPolicy.h`` by this cross-language
golden-vector test: :func:`resolve_chain_speeds` must reproduce every vector in
``tests/fixtures/chain_speed_goldens.json`` BYTE-IDENTICALLY under float32
emulation.

The fixture is the "executable tether" (T4): the host golden dumper
(``chain_speed_dump.cpp``) is driven twice by ``chain_speed_freshness.cmake``
(once to a temp file, once already committed) and byte-compared, so a firmware
retune of ``TSHAPE_ACCEL_PWM_PER_CM`` / ``PWM_MIN_MAGNITUDE`` or a change to the
policy math changes the fixture and fails CI until regenerated. This module
reads the SAME committed fixture and the SAME ``Config::*`` knobs the dumper
built its ``ChainSpeedConfig`` from (via :func:`load_chain_speed_config`), so
the two mirrors stay locked together.

Covered cases (the eight curated vectors, per the dumper's ``buildGoldenCases``):

* **even chain** ``[10,10,10]`` — cruise plateau through the run, floor at end.
* **short FINAL hop** ``[10,10,3]`` — the shed pulled into the prior hop.
* **short MID hop** ``[10,3,10]`` — mid hop too short to dip; stays at cruise.
* **reverse cruise** ``[10,10,10]`` — the negative sign carried into every output.
* **entry == cruise** ``[10,10,10]`` — AC5: no re-dip to the floor at the start.
* **single-hop** ``[10]`` (``hopCount < 2``) — TC1 rejection: EMPTY ``out``.
* **long even chain** ``[40,40,40]`` — accel/decel both fully fit; interior cruise.
* **forward-limited short lead** ``[3,3,20]`` — the FORWARD accel pass BINDS two
  interior nodes (65, 80) below cruise; deleting the forward pass resolves them to
  cruise (85) instead, so this vector is the regression guard for a decel-only
  regression that would otherwise stay byte-green.

In addition to the byte-identity check, :func:`test_all_goldens_respect_accel_bound_both_directions`
asserts a STRUCTURAL two-pass property over every golden vector — every hop's
commanded speed change obeys the acceleration bound in BOTH directions — so
neither a missing forward pass (an exit exceeding ``entry + accel*hopDist``) nor a
too-steep decel can slip through byte-green even if a future golden happened not to
bind the forward pass.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from tools.path_planner.ctps.chain_speed import (
    ChainHopSpeed,
    load_chain_speed_config,
    resolve_chain_speeds,
)
from tools.path_planner.ctps.profile import _f32

FIXTURES_DIR = Path(__file__).resolve().parent / "fixtures"
GOLDENS_PATH = FIXTURES_DIR / "chain_speed_goldens.json"


def _load_goldens() -> list[dict]:
    """Load the committed host golden vectors (T4's ctest-regenerated fixture)."""
    with GOLDENS_PATH.open(encoding="utf-8") as fh:
        return json.load(fh)


GOLDEN_VECTORS = _load_goldens()


# ---------------------------------------------------------------------------
# Fixture sanity — pin the shape so a silently-truncated fixture fails loud
# ---------------------------------------------------------------------------


def test_goldens_fixture_has_eight_vectors_incl_reverse_and_single_hop() -> None:
    """Sanity-pin the fixture: eight vectors, including the reverse-cruise run
    (negative cruise/entry -> negative outputs), the single-hop rejection
    (``hopCount < 2`` -> EMPTY ``out``), and the forward-limited short-lead run
    (``[3,3,20]``) whose FORWARD pass binds two interior nodes below cruise — the
    load-bearing edge cases."""
    assert len(GOLDEN_VECTORS) == 8

    reverse_vectors = [v for v in GOLDEN_VECTORS if v["cruise"] < 0 and v["entry"] < 0]
    assert len(reverse_vectors) >= 1, "expected at least one reverse-cruise vector"

    single_hop_vectors = [v for v in GOLDEN_VECTORS if v["hopCount"] < 2]
    assert len(single_hop_vectors) == 1, "expected exactly one single-hop rejection vector"
    assert single_hop_vectors[0]["out"] == [], "single-hop vector must emit an EMPTY out"

    forward_limited = [v for v in GOLDEN_VECTORS if v["label"] == "forward_limited_short_lead"]
    assert len(forward_limited) == 1, "expected the forward-limited short-lead guard vector"
    # The forward pass must genuinely BIND at least one interior node BELOW cruise; if every
    # interior speed were at cruise, deleting the forward pass would reproduce the vector and the
    # guard would be inert (the exact "green tests, wrong motion" trap this vector closes).
    fl = forward_limited[0]
    interior_start_pwms = [hop["startPwm"] for hop in fl["out"][1:]]
    assert any(abs(pwm) < abs(fl["cruise"]) for pwm in interior_start_pwms), (
        "forward_limited_short_lead must bind an interior node below cruise so the "
        "forward-pass regression guard is not inert"
    )


# ---------------------------------------------------------------------------
# AC6 / TC1 — byte-identity vs the host goldens under float32 emulation
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "vector",
    GOLDEN_VECTORS,
    ids=[v["label"] for v in GOLDEN_VECTORS],
)
def test_resolve_chain_speeds_matches_golden_exactly(vector: dict) -> None:
    """``resolve_chain_speeds`` reproduces every golden ``(startPwm, endPwm, brake)``
    bit-for-bit.

    Runs the float32 mirror over the golden vector's own
    ``(hopDist, hopCount, cruise, entry)`` columns with the SAME ``Config::*``
    knobs the host dumper built its ``ChainSpeedConfig`` from (parsed from the
    real firmware headers via :func:`load_chain_speed_config`, not hardcoded, so
    a retune of ``TSHAPE_ACCEL_PWM_PER_CM`` / ``PWM_MIN_MAGNITUDE`` stays
    consistent with this test). Asserts the produced list is byte-identical to
    the golden ``out`` — same ints, same brake bools, empty for the single-hop
    case.
    """
    cfg = load_chain_speed_config()

    resolved = resolve_chain_speeds(
        vector["hopDist"],
        vector["hopCount"],
        vector["cruise"],
        vector["entry"],
        cfg,
    )

    # Reduce to the same plain-dict shape the fixture uses, then compare whole
    # lists so a length mismatch (e.g. a mis-handled single-hop) fails as loudly
    # as a value mismatch.
    got = [
        {"startPwm": hop.start_pwm, "endPwm": hop.end_pwm, "brake": hop.brake}
        for hop in resolved
    ]
    assert got == vector["out"], (
        f"chain-speed mismatch for {vector['label']}: got {got}, "
        f"expected {vector['out']}"
    )


# ---------------------------------------------------------------------------
# Two-pass structural property — the accel bound holds in BOTH directions
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "vector",
    [v for v in GOLDEN_VECTORS if v["hopCount"] >= 2],
    ids=[v["label"] for v in GOLDEN_VECTORS if v["hopCount"] >= 2],
)
def test_all_goldens_respect_accel_bound_both_directions(vector: dict) -> None:
    """Every resolved hop's node-to-node speed change obeys ``accel*hopDist`` in
    BOTH directions (AC4) — the structural guard that complements byte-identity.

    This closes the exact "green tests, wrong motion" trap the plan Notes name
    ("A decel-only formula passes the goldens but ships a steeper-than-``accel``
    slam — Goldens must assert the forward constraint too", TC1). Byte-identity
    alone lets a decel-only (forward-pass-removed) regression stay green on any
    vector whose forward pass never binds; this property fails such a regression
    directly:

    * **Forward (accel-limit):** ``v[k+1] <= v[k] + accel*hopDist[k]``. A missing
      forward pass leaves an interior node ABOVE what the acceleration allows
      from the entry (e.g. resolving ``[3,3,20]``'s node1 to cruise 85 instead of
      the accel-limited 65), which this bound catches.
    * **Backward (decel-to-floor):** ``v[k] <= v[k+1] + accel*hopDist[k]`` — a
      too-steep decel (the half-clamp "slam") violates it in the other direction.

    Node speeds are reconstructed in the magnitude domain from the resolved
    ``(startPwm, endPwm)``: ``v[0] = |out[0].startPwm|``, each interior boundary
    ``v[k] = |out[k].startPwm|`` (== ``|out[k-1].endPwm|``), and the terminal node
    ``v[hopCount] = pwmMin`` — the floor the final hop's ``cruise->floor->brake``
    stop sheds to (the final hop's emitted ``endPwm`` is 0, the brake, NOT the
    node speed, so we substitute the backward-pass floor target instead).

    The tolerance absorbs the int16 truncation-toward-zero applied at the output
    boundary (a resolved integer ``n`` came from a pre-truncation real in
    ``[n, n+1)`` for a forward run) plus float32 noise in the ``accel*hopDist``
    product; ``accel`` and the product are taken through ``_f32`` for the same
    float discipline the planner uses.
    """
    cfg = load_chain_speed_config()
    out = vector["out"]
    hop_count = vector["hopCount"]
    hop_dist = vector["hopDist"]
    accel = _f32(cfg.accel_pwm_per_cm) if cfg.accel_pwm_per_cm > 0.0 else _f32(1e-6)

    # Reconstruct the node-speed magnitude sequence v[0..hopCount] from the
    # resolved outputs. Boundaries between hops are shared: out[k].endPwm ==
    # out[k+1].startPwm for every mid boundary; the final node is the pwmMin floor
    # (the final hop emits endPwm=0, the brake, not the node speed).
    node_mag = [abs(out[k]["startPwm"]) for k in range(hop_count)]
    node_mag.append(cfg.pwm_min)  # terminal node: the floor the run sheds to

    # Sanity: the shared mid boundaries agree (mid-hop endPwm == next startPwm).
    for k in range(hop_count - 1):
        assert abs(out[k]["endPwm"]) == node_mag[k + 1], (
            f"{vector['label']}: hop {k} endPwm |{out[k]['endPwm']}| != next "
            f"startPwm {node_mag[k + 1]} — node boundary inconsistent"
        )

    # One unit absorbs the int16 truncation-toward-zero at the output boundary;
    # a small float32-scaled slack absorbs accel*dist rounding noise.
    tolerance = 1.0 + 1e-3

    for k in range(hop_count):
        v_entry = float(node_mag[k])
        v_exit = float(node_mag[k + 1])
        d = _f32(float(hop_dist[k]))
        if d < 0.0:
            d = _f32(0.0)
        bound = _f32(accel * d)
        delta = abs(v_exit - v_entry)
        assert delta <= bound + tolerance, (
            f"{vector['label']}: hop {k} speed change |{v_exit} - {v_entry}| = "
            f"{delta} exceeds accel*hopDist = {accel}*{hop_dist[k]} = {bound} "
            f"(+tol {tolerance}) — a missing forward pass or a too-steep decel"
        )


def test_single_hop_returns_empty_list() -> None:
    """A ``hop_count < 2`` run is NOT a chain — the mirror returns an empty list,
    leaving the caller to take the ordinary per-hop path (TC1 rejection)."""
    cfg = load_chain_speed_config()
    assert resolve_chain_speeds([10.0], 1, 85, 50, cfg) == []
    assert resolve_chain_speeds([], 0, 85, 50, cfg) == []


def test_reverse_cruise_carries_sign_into_every_output() -> None:
    """A negative ``cruise_pwm`` yields negative ``start_pwm`` / ``end_pwm`` on
    every non-zero setpoint; the final-hop ``end_pwm`` stays exactly 0 (the
    brake), and ``brake`` is set only on the final hop."""
    cfg = load_chain_speed_config()
    resolved = resolve_chain_speeds([10.0, 10.0, 10.0], 3, -85, -50, cfg)

    assert len(resolved) == 3
    for hop in resolved:
        assert hop.start_pwm <= 0, "reverse run must not command a positive start"
        assert hop.end_pwm <= 0, "reverse run must not command a positive end"
    assert [hop.brake for hop in resolved] == [False, False, True]
    assert resolved[-1].end_pwm == 0, "final hop end speed is 0 (the brake)"


def test_default_cfg_matches_explicit_cfg() -> None:
    """Omitting ``cfg`` resolves the same firmware-header config as passing it
    explicitly, so a caller (T8 sim / T9 CTPS) may rely on the default."""
    explicit = load_chain_speed_config()
    with_default = resolve_chain_speeds([10.0, 10.0, 10.0], 3, 85, 50)
    with_explicit = resolve_chain_speeds([10.0, 10.0, 10.0], 3, 85, 50, explicit)
    assert with_default == with_explicit


def test_hop_speed_is_hashable_frozen() -> None:
    """``ChainHopSpeed`` is a frozen dataclass (immutable value type), so a
    resolved hop can be used as a dict key / set member if a caller needs it."""
    hop = ChainHopSpeed(start_pwm=85, end_pwm=0, brake=True)
    with pytest.raises((AttributeError, TypeError)):
        hop.start_pwm = 50  # type: ignore[misc]
    assert hop == ChainHopSpeed(start_pwm=85, end_pwm=0, brake=True)
