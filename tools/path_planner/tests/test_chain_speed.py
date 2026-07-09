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

Covered cases (the seven curated vectors, per the dumper's ``buildGoldenCases``):

* **even chain** ``[10,10,10]`` — cruise plateau through the run, floor at end.
* **short FINAL hop** ``[10,10,3]`` — the shed pulled into the prior hop.
* **short MID hop** ``[10,3,10]`` — mid hop too short to dip; stays at cruise.
* **reverse cruise** ``[10,10,10]`` — the negative sign carried into every output.
* **entry == cruise** ``[10,10,10]`` — AC5: no re-dip to the floor at the start.
* **single-hop** ``[10]`` (``hopCount < 2``) — TC1 rejection: EMPTY ``out``.
* **long even chain** ``[40,40,40]`` — accel/decel both fully fit; interior cruise.
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


def test_goldens_fixture_has_seven_vectors_incl_reverse_and_single_hop() -> None:
    """Sanity-pin the fixture: seven vectors, including the reverse-cruise run
    (negative cruise/entry -> negative outputs) and the single-hop rejection
    (``hopCount < 2`` -> EMPTY ``out``), the two load-bearing edge cases."""
    assert len(GOLDEN_VECTORS) == 7

    reverse_vectors = [v for v in GOLDEN_VECTORS if v["cruise"] < 0 and v["entry"] < 0]
    assert len(reverse_vectors) >= 1, "expected at least one reverse-cruise vector"

    single_hop_vectors = [v for v in GOLDEN_VECTORS if v["hopCount"] < 2]
    assert len(single_hop_vectors) == 1, "expected exactly one single-hop rejection vector"
    assert single_hop_vectors[0]["out"] == [], "single-hop vector must emit an EMPTY out"


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
