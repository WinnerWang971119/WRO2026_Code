"""Tests for the CTPS firmware-faithful motion-profile kernel (T6: TC1-TC5).

``tools.path_planner.ctps.profile`` is the single Python source of the T-shape
velocity profile the CTPS player renders (2026-07-09 CTPS motion-profile
plan). It is pinned to the firmware ``TShapeProfilePolicy.h`` by a
cross-language golden-vector test; this module is the executable tether
(AC9) plus the regime/shape/perturbation coverage the plan's Testing
Strategy calls TC1-TC5.

Covered cases:

* **TC1** — ``resolve_tshape_ramps`` / ``tshape_speed_at`` reproduce every
  vector in ``tests/fixtures/tshape_profile_goldens.json`` bit-for-bit under
  float32 emulation, including the reverse-drive vector (vector index 3) and
  the two sign-clamp vectors (indices 9-10, negative-cruise and
  positive-cruise saturating start/end).
* **TC2** — doubling ``TSHAPE_ACCEL_PWM_PER_CM`` via the ``config_path`` temp-
  header seam halves the resolved ramp distances/durations (AC4).
* **TC3** — the 3 cm / 12 cm / 40 cm regime boundaries classify as flat /
  triangular / trapezoid (AC1), using ``DriveProfile.regime`` and the
  ``|peak| < |cruise|`` triangular signature (NOT ``cruise_cm == 0`` — the
  12 cm case leaves a small residual plateau under the asymmetric half-cap).
* **TC4** — an ``endSpeed=0`` stop renders a finite duration with a
  floor-crawl tail at ``PWM_MIN_MAGNITUDE`` before the terminal brake (AC2).
* **TC5** — ``resolve_effective_speed`` on a ``.setBrake(false)`` command
  defaults ``end`` to ``cruise``, not 0 (no phantom decel-to-zero).
* **TC6** — the DISPLAY-ONLY constant-rate + settle-allowance turn profile
  (ADR-068 T8; the two-stage slow-zone-angle / slow-stage-PWM knobs it used to
  mirror no longer exist, AC7). (a) Under the DEFAULT Config a turn/pivot
  sweeps at a SINGLE constant rate derived from ``TURN_CRUISE_PWM = 50``
  (unsnapped — the real firmware turn path no longer applies ``deadbandSnap``
  to any turn command, AC3), so every ``sample_turn`` sub-keyframe interval is
  evenly timed EXCEPT the last, which also carries the fixed nominal
  ``TURN_SETTLE_ALLOWANCE_S`` tail and therefore renders strictly slower — a
  stand-in for the PD's creep-and-brake phase, not a second swept-angle stage.
  (b) A perturbed config (``TURN_CRUISE_PWM = 80`` via the ``config_path``
  temp-header seam) raises the constant rate in exact proportion to the PWM
  ratio; the fixed settle allowance is unaffected by Config (TC19).
* **W1** — ``resolve_effective_speed`` on a raw ``driveDist`` with a NEGATIVE
  ``.setPower`` keeps the sign: cruise resolves negative and the start seed
  follows sign-by-cruise (a reverse ramp), NOT ``abs()``-ed to forward. A
  positive power still resolves forward (no regression).
* **TC9** (ADR-059 / collinear-run, AC8) — the LineFollower window-fit in the
  stop kernel: a FAST (cruise 90) + REAR-led braked final hop whose natural
  ``peak -> floor`` shed (8 cm) OVERRUNS the rear overshoot window
  (``rear_axle_offset_cm`` ~= 7.68 cm) renders the WINDOW-FITTED shed — a
  steeper decel that reaches the floor AT the gate (``decel_cm <= window``,
  ``floor_crossing_cm == dist``, ``floor_tail_cm == 0``, ``|end_pwm| ==
  PWM_MIN``). A FRONT-window control (12.02 cm) does NOT bind that 8 cm shed, so
  the natural stop is unchanged (``floor_tail_cm > 0``, ``end_pwm == 0``). No
  build-failing assert — a render requirement only (the firmware degrades
  gracefully).
"""

from __future__ import annotations

import json
import shutil
from math import degrees
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from tools.path_planner.config_parse import DEFAULT_CONFIG_H_PATH
from tools.path_planner.ctps.config import TURN_SETTLE_ALLOWANCE_S
from tools.path_planner.ctps.profile import (
    SCALE_CM_S_PER_PWM,
    ApproachPhase,
    ProfileConfig,
    approach_speed_at,
    default_config,
    load_profile_config,
    resolve_approach_schedule,
    resolve_drive_profile,
    resolve_effective_speed,
    resolve_tshape_ramps,
    resolve_turn_profile,
    sample_turn,
    tshape_speed_at,
)

FIXTURES_DIR = Path(__file__).resolve().parent / "fixtures"
GOLDENS_PATH = FIXTURES_DIR / "tshape_profile_goldens.json"
APPROACH_GOLDENS_PATH = FIXTURES_DIR / "approach_ramp_goldens.json"

# The sibling headers profile.py's config loader resolves relative to
# `config_path.parent` (ConfigDrive.h itself + ConfigGlobal.h + ConfigLine.h +
# ConfigNav.h). ConfigNav.h is required for HOP_SETTLE_MS resolution, so a
# perturbation test's temp dir must hold it too or load_profile_config raises.
_CONFIG_HEADER_NAMES = ("ConfigDrive.h", "ConfigGlobal.h", "ConfigLine.h", "ConfigNav.h")


def _load_goldens() -> list[dict]:
    """Load the committed host golden vectors (T3's ctest-regenerated fixture)."""
    with GOLDENS_PATH.open(encoding="utf-8") as fh:
        return json.load(fh)


GOLDEN_VECTORS = _load_goldens()


def _load_approach_goldens() -> list[dict]:
    """Load the committed approach-ramp golden vectors (T3's ``approach_ramp_dump`` fixture)."""
    with APPROACH_GOLDENS_PATH.open(encoding="utf-8") as fh:
        return json.load(fh)


APPROACH_GOLDEN_VECTORS = _load_approach_goldens()


# ---------------------------------------------------------------------------
# TC1 — parity vs host goldens (bit-for-bit under float32 emulation, AC9)
# ---------------------------------------------------------------------------


def test_goldens_fixture_has_eleven_vectors_incl_reverse_and_clamp_cases() -> None:
    """Sanity-pin the fixture shape: 11 vectors, including the reverse hop
    (negative cruise/start/end) and the two sign-clamp cases (index 9:
    negative-cruise saturating start=end=-90; index 10: positive-cruise
    saturating start=end=90) that T3/T5 flagged as load-bearing."""
    assert len(GOLDEN_VECTORS) == 11

    reverse_vectors = [
        v for v in GOLDEN_VECTORS if v["cruise"] < 0 and v["start"] < 0 and v["end"] <= 0
    ]
    assert len(reverse_vectors) >= 1, "expected at least one reverse-hop vector"

    clamp_vectors = [
        v
        for v in GOLDEN_VECTORS
        if v["start"] == v["end"] and abs(v["start"]) > abs(v["cruise"])
    ]
    assert len(clamp_vectors) == 2, "expected exactly two sign-clamp vectors"
    clamp_signs = {1 if v["cruise"] > 0 else -1 for v in clamp_vectors}
    assert clamp_signs == {1, -1}, "sign-clamp vectors must cover both signs"


@pytest.mark.parametrize(
    "vector",
    GOLDEN_VECTORS,
    ids=[
        f"dist={v['dist']}_cruise={v['cruise']}_start={v['start']}_end={v['end']}"
        for v in GOLDEN_VECTORS
    ],
)
def test_resolve_tshape_ramps_matches_golden_exactly(vector: dict) -> None:
    """``resolve_tshape_ramps`` reproduces accelCm/decelCm/peakPwm bit-for-bit.

    Calls the pure kernel directly with the golden vector's own
    ``(dist, cruise, start, end, accel)`` columns and the REAL firmware
    ramp_min/ramp_max/no_ramp_eps pulled from ``profile.py``'s config loader
    (not hardcoded), so a retune of those knobs stays consistent with this
    test. float32 comparison is exact (both sides already round through
    ``numpy.float32`` inside the kernel).
    """
    cfg = load_profile_config()

    accel_cm, decel_cm, peak_pwm = resolve_tshape_ramps(
        vector["dist"],
        vector["cruise"],
        vector["start"],
        vector["end"],
        vector["accel"],
        cfg.ramp_min,
        cfg.ramp_max,
        cfg.no_ramp_eps,
    )

    expect = vector["expect"]
    assert np.float32(accel_cm) == np.float32(expect["accelCm"]), (
        f"accelCm mismatch for {vector}: got {accel_cm}, expected {expect['accelCm']}"
    )
    assert np.float32(decel_cm) == np.float32(expect["decelCm"]), (
        f"decelCm mismatch for {vector}: got {decel_cm}, expected {expect['decelCm']}"
    )
    assert int(peak_pwm) == int(expect["peakPwm"]), (
        f"peakPwm mismatch for {vector}: got {peak_pwm}, expected {expect['peakPwm']}"
    )


@pytest.mark.parametrize(
    "vector",
    GOLDEN_VECTORS,
    ids=[
        f"dist={v['dist']}_cruise={v['cruise']}_start={v['start']}_end={v['end']}"
        for v in GOLDEN_VECTORS
    ],
)
def test_tshape_speed_at_matches_golden_pwm_at_samples(vector: dict) -> None:
    """Every ``pwm_at`` sample reproduces its exact int PWM via ``tshape_speed_at``.

    Re-resolves the ramps for this vector (mirroring how a caller would
    chain the two kernel calls: resolve once, then sample speed-at along the
    move), then asserts each golden ``{traveled, pwm}`` pair exactly.
    """
    cfg = load_profile_config()

    accel_cm, decel_cm, peak_pwm = resolve_tshape_ramps(
        vector["dist"],
        vector["cruise"],
        vector["start"],
        vector["end"],
        vector["accel"],
        cfg.ramp_min,
        cfg.ramp_max,
        cfg.no_ramp_eps,
    )

    for sample in vector["expect"]["pwm_at"]:
        pwm = tshape_speed_at(
            sample["traveled"],
            vector["dist"],
            accel_cm,
            decel_cm,
            peak_pwm,
            vector["start"],
            vector["end"],
        )
        assert int(pwm) == int(sample["pwm"]), (
            f"pwm_at mismatch for {vector} at traveled={sample['traveled']}: "
            f"got {pwm}, expected {sample['pwm']}"
        )


def test_resolve_drive_profile_pwm_at_s_matches_golden_pwm_at_samples() -> None:
    """The Config-wired ``DriveProfile.pwm_at_s`` agrees with the same goldens.

    TC1 asks for the golden ``pwm_at`` samples to be checked "via
    profile.tshape_speed_at or the DriveProfile's pwm_at_s, whichever the
    goldens' traveled maps to" — this cross-checks BOTH call paths agree,
    using the real (non-perturbed) firmware config so the wrapped resolver
    exercises the same accel/ramp/eps values as the bare kernel calls above.

    Only vectors whose ``dist`` clears ``TSHAPE_MIN_MOVE_CM`` (5 cm) are
    routed through ``resolve_drive_profile``: the min-move flat-cruise gate
    (AC1's regime rule) lives ONLY in that Config-wired wrapper, not in the
    bare ``resolve_tshape_ramps``/``tshape_speed_at`` kernel the golden
    vectors pin directly — the 3 cm golden vector is a bare-kernel fixture
    (TC1 above already covers it via the un-gated kernel calls) and correctly
    degrades to flat cruise through this wrapper, which is TC3's concern, not
    this cross-check's.
    """
    cfg = load_profile_config()
    for vector in GOLDEN_VECTORS:
        if vector["dist"] < cfg.min_move_cm:
            continue
        profile = resolve_drive_profile(
            vector["dist"],
            cruise_pwm=vector["cruise"],
            start_pwm=vector["start"],
            end_pwm=vector["end"],
            profiled=True,
        )
        assert np.float32(profile.accel_cm) == np.float32(vector["expect"]["accelCm"])
        assert np.float32(profile.decel_cm) == np.float32(vector["expect"]["decelCm"])
        assert int(profile.peak_pwm) == int(vector["expect"]["peakPwm"])

        for sample in vector["expect"]["pwm_at"]:
            pwm = profile.pwm_at_s(sample["traveled"])
            assert int(pwm) == int(sample["pwm"]), (
                f"DriveProfile.pwm_at_s mismatch for {vector} at "
                f"traveled={sample['traveled']}: got {pwm}, expected {sample['pwm']}"
            )


# ---------------------------------------------------------------------------
# TC13 (AC10) — approach-ramp golden parity (T5)
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "vector",
    APPROACH_GOLDEN_VECTORS,
    ids=[v["label"] for v in APPROACH_GOLDEN_VECTORS],
)
def test_resolve_approach_schedule_matches_golden_exactly(vector: dict) -> None:
    """``resolve_approach_schedule`` reproduces the golden schedule bit-for-bit.

    Calls the pure kernel directly with the golden vector's own
    ``(predictedCm, marginCm, maxMarginCm, cruiseMag, floorMag, accel, rampMin,
    rampMax, noRampEps, minMoveCm)`` columns (the C++ dump's exact resolve
    inputs) — mirrors how TC1's
    ``test_resolve_tshape_ramps_matches_golden_exactly`` calls
    ``resolve_tshape_ramps`` directly with its vector's own columns rather than
    routing through the real firmware Config. float32 comparison is exact
    (both sides already round through ``numpy.float32`` inside the kernel, and
    the C++ dump's ``%.9g`` formatting round-trips a float32 exactly).
    """
    schedule = resolve_approach_schedule(
        vector["predictedCm"],
        vector["marginCm"],
        vector["maxMarginCm"],
        vector["cruiseMag"],
        vector["floorMag"],
        vector["accel"],
        vector["rampMin"],
        vector["rampMax"],
        vector["noRampEps"],
        vector["minMoveCm"],
    )

    expect = vector["expect"]
    assert np.float32(schedule.ramp_start_cm) == np.float32(expect["rampStartCm"]), (
        f"rampStartCm mismatch for {vector}: got {schedule.ramp_start_cm}, "
        f"expected {expect['rampStartCm']}"
    )
    assert np.float32(schedule.floor_start_cm) == np.float32(expect["floorStartCm"]), (
        f"floorStartCm mismatch for {vector}: got {schedule.floor_start_cm}, "
        f"expected {expect['floorStartCm']}"
    )
    assert np.float32(schedule.overrun_limit_cm) == np.float32(expect["overrunLimitCm"]), (
        f"overrunLimitCm mismatch for {vector}: got {schedule.overrun_limit_cm}, "
        f"expected {expect['overrunLimitCm']}"
    )
    assert int(schedule.cruise_mag) == int(expect["cruiseMag"]), (
        f"cruiseMag mismatch for {vector}: got {schedule.cruise_mag}, "
        f"expected {expect['cruiseMag']}"
    )
    assert int(schedule.floor_mag) == int(expect["floorMag"]), (
        f"floorMag mismatch for {vector}: got {schedule.floor_mag}, "
        f"expected {expect['floorMag']}"
    )
    assert bool(schedule.active) == bool(expect["active"]), (
        f"active mismatch for {vector}: got {schedule.active}, "
        f"expected {expect['active']}"
    )


@pytest.mark.parametrize(
    "vector",
    APPROACH_GOLDEN_VECTORS,
    ids=[v["label"] for v in APPROACH_GOLDEN_VECTORS],
)
def test_approach_ramp_golden(vector: dict) -> None:
    """TC13 (AC10): ``approach_speed_at`` reproduces every golden ``speed_at``
    sample's exact ``(pwmMag, phase)``.

    Re-resolves the schedule for this vector (mirroring how a caller chains
    the two kernel calls: resolve once, then sample speed-at along the hop),
    then asserts each golden ``{traveled, pwmMag, phase}`` triple exactly,
    mapping the golden's phase string onto :class:`ApproachPhase` by name. A
    degenerate/inactive schedule (T3's dump emitted an empty ``speed_at`` list
    for those cases: short-hop-below-min-move and ramp-start-non-positive)
    asserts ``active is False`` and skips the sample sweep — sampling a
    schedule whose boundaries are not load-bearing is meaningless
    (``resolve_approach_schedule``'s own docstring: "the caller must not
    invoke [``approach_speed_at``] on a schedule with active == False").
    """
    schedule = resolve_approach_schedule(
        vector["predictedCm"],
        vector["marginCm"],
        vector["maxMarginCm"],
        vector["cruiseMag"],
        vector["floorMag"],
        vector["accel"],
        vector["rampMin"],
        vector["rampMax"],
        vector["noRampEps"],
        vector["minMoveCm"],
    )

    expect = vector["expect"]
    if not expect["active"]:
        assert schedule.active is False
        assert expect["speed_at"] == []
        return

    for sample in expect["speed_at"]:
        speed = approach_speed_at(schedule, sample["traveled"])
        expected_phase = ApproachPhase[sample["phase"]]
        assert int(speed.pwm_mag) == int(sample["pwmMag"]), (
            f"pwmMag mismatch for {vector} at traveled={sample['traveled']}: "
            f"got {speed.pwm_mag}, expected {sample['pwmMag']}"
        )
        assert speed.phase == expected_phase, (
            f"phase mismatch for {vector} at traveled={sample['traveled']}: "
            f"got {speed.phase!r}, expected {expected_phase!r}"
        )


# ---------------------------------------------------------------------------
# TC2 — Config perturbation (AC4)
# ---------------------------------------------------------------------------


def _copy_config_headers(dest_dir: Path) -> Path:
    """Copy the three Config sub-headers profile.py resolves into ``dest_dir``.

    ``load_profile_config`` resolves ``ConfigGlobal.h`` / ``ConfigLine.h``
    relative to ``config_path.parent``, so ALL THREE headers must land in the
    same temp directory for a perturbed ``ConfigDrive.h`` to still resolve
    its siblings (PWM_MIN_MAGNITUDE, CRUISE_SPEED, etc.).

    :param dest_dir: temp directory to copy the headers into.
    :return: the path to the copied ``ConfigDrive.h`` (the ``config_path``
        seam a caller perturbs before re-parsing).
    """
    config_dir = DEFAULT_CONFIG_H_PATH.parent
    for name in _CONFIG_HEADER_NAMES:
        shutil.copy2(config_dir / name, dest_dir / name)
    return dest_dir / "ConfigDrive.h"


def test_doubling_tshape_accel_halves_ramp_distances_and_durations(tmp_path: Path) -> None:
    """Doubling ``TSHAPE_ACCEL_PWM_PER_CM`` 4->8 halves ramp distances/durations.

    Copies all three Config sub-headers into a temp dir (the ``config_path``
    seam), doubles ``TSHAPE_ACCEL_PWM_PER_CM`` in the copied ``ConfigDrive.h``
    only, and re-parses via ``load_profile_config`` pointed at the temp
    header. A steeper ramp (double PWM-per-cm) covers the same PWM delta in
    half the distance, and the closed-form duration for that halved ramp
    shortens too, so the whole move's render duration changes (AC4).
    """
    baseline_cfg = load_profile_config()
    assert baseline_cfg.accel == pytest.approx(4.0)

    perturbed_dir = tmp_path / "perturbed_config"
    perturbed_dir.mkdir()
    perturbed_drive_h = _copy_config_headers(perturbed_dir)

    original_text = perturbed_drive_h.read_text(encoding="utf-8")
    doubled_text = original_text.replace(
        "constexpr float TSHAPE_ACCEL_PWM_PER_CM = 4.0f;",
        "constexpr float TSHAPE_ACCEL_PWM_PER_CM = 8.0f;",
    )
    assert doubled_text != original_text, (
        "the literal TSHAPE_ACCEL_PWM_PER_CM declaration text did not match; "
        "ConfigDrive.h's declaration shape changed upstream"
    )
    perturbed_drive_h.write_text(doubled_text, encoding="utf-8")

    perturbed_cfg = load_profile_config(perturbed_drive_h)
    assert perturbed_cfg.accel == pytest.approx(8.0)

    # A representative trapezoid move (dist=40, cruise=90, start=35, end=35):
    # ramp distance = |delta| / accel, so doubling accel halves BOTH ramps. The
    # move starts and ends AT the PWM floor (so no sub-floor decel-tail crawl
    # muddies the decel ramp) and the delta is wide enough that even the
    # doubled-accel ramps stay above TSHAPE_RAMP_MIN_CM — the halving is visible
    # and neither floor-clamped nor floor-tailed.
    baseline_profile = resolve_drive_profile(
        40.0, cruise_pwm=90, start_pwm=35, end_pwm=35, profiled=True, cfg=baseline_cfg
    )
    perturbed_profile = resolve_drive_profile(
        40.0, cruise_pwm=90, start_pwm=35, end_pwm=35, profiled=True, cfg=perturbed_cfg
    )

    assert perturbed_profile.accel_cm == pytest.approx(baseline_profile.accel_cm / 2.0)
    assert perturbed_profile.decel_cm == pytest.approx(baseline_profile.decel_cm / 2.0)

    # Render duration changes as a consequence of the shortened ramps.
    assert perturbed_profile.duration_s() != pytest.approx(baseline_profile.duration_s())
    assert perturbed_profile.duration_s() < baseline_profile.duration_s()


def test_config_perturbation_seam_copies_all_three_headers(tmp_path: Path) -> None:
    """Regression guard: omitting a sibling header breaks the seam loudly.

    If only ``ConfigDrive.h`` is copied into the temp dir (not its two
    siblings), ``load_profile_config`` must fail loud (issue #44 policy)
    rather than silently falling back — this pins WHY all three headers are
    required, not just documents it.
    """
    lonely_dir = tmp_path / "lonely_config"
    lonely_dir.mkdir()
    shutil.copy2(DEFAULT_CONFIG_H_PATH, lonely_dir / "ConfigDrive.h")

    with pytest.raises(OSError):
        load_profile_config(lonely_dir / "ConfigDrive.h")


# ---------------------------------------------------------------------------
# TC3 — Regime boundaries: 3 cm / 12 cm / 40 cm (AC1)
# ---------------------------------------------------------------------------


def test_3cm_move_renders_flat_cruise_no_ramp() -> None:
    """A move below TSHAPE_MIN_MOVE_CM (5 cm) degrades to flat cruise.

    Firmware-faithful (plan Decisions): ``Drive.h`` gates the profiled move
    on ``_cm >= TSHAPE_MIN_MOVE_CM``; below that it enqueues a flat
    ``DRIVE_DIST`` with no ramp at all.
    """
    profile = resolve_drive_profile(
        3.0, cruise_pwm=85, start_pwm=50, end_pwm=0, profiled=True
    )
    assert profile.regime == "flat"
    assert profile.accel_cm == 0.0
    assert profile.decel_cm == 0.0
    assert profile.peak_pwm == 85


def test_40cm_move_renders_full_trapezoid_with_genuine_plateau() -> None:
    """A move >= accel+decel is a full trapezoid with a genuine cruise plateau."""
    profile = resolve_drive_profile(
        40.0, cruise_pwm=85, start_pwm=50, end_pwm=0, profiled=True
    )
    assert profile.regime == "trapezoid"
    assert profile.triangular is False
    assert profile.peak_pwm == profile.cruise_pwm
    assert profile.cruise_cm > 0.0


def test_12cm_move_renders_triangular_with_reduced_peak_below_cruise() -> None:
    """A 12 cm move (cruise 85, start 50, end 0) is triangular.

    IMPORTANT (T5 review finding): the asymmetric half-of-total cap leaves a
    SMALL residual plateau at the reduced peak (``cruise_cm`` is not exactly
    0), so the correct triangular assertion is ``regime == "triangular"``
    AND ``|peak| < |cruise|`` (peak reduced below cruise, no FULL-peak
    plateau) — NOT ``cruise_cm == 0``.
    """
    profile = resolve_drive_profile(
        12.0, cruise_pwm=85, start_pwm=50, end_pwm=0, profiled=True
    )
    assert profile.regime == "triangular"
    assert profile.triangular is True
    assert abs(profile.peak_pwm) < abs(profile.cruise_pwm)
    # Pin the T5-review-documented residual: peak == 45, a small plateau
    # remains (cruise_cm ~ 2.5), not a fully-collapsed zero-plateau triangle.
    assert profile.peak_pwm == 45
    assert profile.cruise_cm > 0.0


# ---------------------------------------------------------------------------
# TC4 — Stop shape at end=0 (AC2)
# ---------------------------------------------------------------------------


def test_stop_at_end_speed_zero_has_finite_duration() -> None:
    """A profiled stop (endSpeed=0) has a finite, sane duration — no divergence.

    The naive ``dt = ds/(k*pwm)`` clock diverges logarithmically as PWM ->
    0; the closed-form deadband-floor model must NOT reproduce that.
    """
    profile = resolve_drive_profile(
        40.0, cruise_pwm=85, start_pwm=50, end_pwm=0, profiled=True
    )
    duration = profile.duration_s()
    assert duration > 0.0
    assert duration < float("inf")
    # Sanity bound: a 40 cm move at a display scale of ~0.35 cm/s-per-PWM and
    # cruise 85 should land well under a minute of on-screen time.
    assert duration < 60.0


def test_stop_at_end_speed_zero_has_floor_crawl_tail_before_dist_cm() -> None:
    """The decel ramp reaches the kinetic decel-tail floor (``PWM_MOVING_MIN``)
    strictly before ``dist_cm``, leaving a floor-crawl tail — the visible
    "crawl then brake" shape (AC7)."""
    cfg = load_profile_config()
    profile = resolve_drive_profile(
        40.0, cruise_pwm=85, start_pwm=50, end_pwm=0, profiled=True, cfg=cfg
    )

    assert profile.floor_tail_cm > 0.0
    assert profile.floor_crossing_cm < profile.dist_cm
    assert profile.floor_crossing_cm + profile.floor_tail_cm == pytest.approx(
        profile.dist_cm
    )

    # The PWM setpoint at the floor crossing is exactly the KINETIC decel-tail
    # floor (PWM_MOVING_MIN) — the decel ramp floors lower than the static
    # breakaway floor (pwm_min) once in the profile decel region — and it stays
    # pinned at that floor through the crawl tail up to (but not including a
    # further drop past) dist_cm.
    pwm_at_crossing = profile.pwm_at_s(profile.floor_crossing_cm)
    assert abs(pwm_at_crossing) == pytest.approx(cfg.pwm_moving_min, abs=1)


def test_stop_at_end_speed_zero_no_divide_by_zero_at_boundaries() -> None:
    """Evaluating the profile at s=0 and s=dist_cm never raises (no div-by-zero)."""
    profile = resolve_drive_profile(
        40.0, cruise_pwm=85, start_pwm=50, end_pwm=0, profiled=True
    )
    assert profile.pwm_at_s(0.0) == 50
    assert profile.pwm_at_s(profile.dist_cm) == 0
    assert profile.velocity_at_s(0.0) > 0.0
    assert profile.velocity_at_s(profile.dist_cm) > 0.0


def test_zero_length_move_has_zero_duration_no_divide_by_zero() -> None:
    """A degenerate zero-distance move resolves without raising and is 0 s."""
    profile = resolve_drive_profile(
        0.0, cruise_pwm=85, start_pwm=50, end_pwm=0, profiled=True
    )
    assert profile.duration_s() == 0.0


# ---------------------------------------------------------------------------
# TC5 — `.setBrake(false)` move: default end_speed resolves to cruise
# ---------------------------------------------------------------------------


def test_brake_false_defaults_end_speed_to_cruise_not_zero() -> None:
    """A ``.setBrake(false)`` command's default ``end`` resolves to ``cruise``.

    Per the plan Decisions: ``.setBrake(false)`` defaults ``endSpeed`` to
    cruise (not 0), so a no-brake / chained move renders no phantom
    decel-to-zero. Build a duck-typed command stub (mirrors a loaded
    ``Command`` / a plain dict) with ``brake=False`` and no explicit
    ``end_speed``.
    """
    cmd = SimpleNamespace(
        power=None,
        preset=None,
        start_speed=None,
        end_speed=None,
        profiled=None,
        brake=False,
        reverse=False,
    )
    cruise, start, end, profiled = resolve_effective_speed(cmd, "driveDist")

    cfg = load_profile_config()
    assert cruise == cfg.speed_normal
    assert end == cruise
    assert end != 0
    assert profiled is True


def test_brake_true_or_unset_defaults_end_speed_to_zero() -> None:
    """Contrast case: brake True (or unset) keeps the default stop-at-rest end=0."""
    cmd_brake_true = SimpleNamespace(
        power=None,
        preset=None,
        start_speed=None,
        end_speed=None,
        profiled=None,
        brake=True,
        reverse=False,
    )
    _, _, end_true, _ = resolve_effective_speed(cmd_brake_true, "driveDist")
    assert end_true == 0

    cmd_brake_unset = SimpleNamespace(
        power=None,
        preset=None,
        start_speed=None,
        end_speed=None,
        profiled=None,
        brake=None,
        reverse=False,
    )
    _, _, end_unset, _ = resolve_effective_speed(cmd_brake_unset, "driveDist")
    assert end_unset == 0


def test_brake_false_with_explicit_end_speed_override_wins() -> None:
    """An explicit ``end_speed`` overrides the brake=false default-to-cruise rule."""
    cmd = SimpleNamespace(
        power=None,
        preset=None,
        start_speed=None,
        end_speed=30,
        profiled=None,
        brake=False,
        reverse=False,
    )
    _, _, end, _ = resolve_effective_speed(cmd, "driveDist")
    assert end == 30


def test_brake_false_dict_style_command_also_resolves_end_to_cruise() -> None:
    """``resolve_effective_speed`` duck-types a plain dict command too."""
    cmd = {
        "power": None,
        "preset": "fast",
        "start_speed": None,
        "end_speed": None,
        "profiled": None,
        "brake": False,
        "reverse": False,
    }
    cruise, start, end, profiled = resolve_effective_speed(cmd, "driveDist")

    cfg = load_profile_config()
    assert cruise == cfg.speed_fast
    assert end == cruise
    assert profiled is True


# ---------------------------------------------------------------------------
# W1 — a negative `.setPower` keeps its sign (a reverse dead-reckon driveDist)
# ---------------------------------------------------------------------------


def test_negative_power_resolves_to_reverse_cruise_and_start() -> None:
    """A raw ``driveDist(20).setPower(-70)`` resolves to a REVERSE profile.

    ``DriveDistBuilder::setPower`` takes a signed ``[-100, 100]`` and has NO
    ``setReverse`` — for a raw dead-reckon ``driveDist`` a negative power IS the
    reverse mechanism (the firmware drives a negative cruise backward). So the
    resolver must keep the sign: ``cruise = -70`` (NOT the old ``abs()``-ed
    ``+70``), and the sign-by-cruise start seed then follows a REVERSE ramp
    (``start = -TSHAPE_START_SPEED``). This is the regression the old
    ``cruise_mag = abs(int(power))`` broke — it rendered the backup as a forward
    move and integrated the operator offset the wrong way.
    """
    cmd = SimpleNamespace(
        power=-70,
        preset=None,
        start_speed=None,
        end_speed=None,
        profiled=None,
        brake=None,
        reverse=False,
    )
    cruise, start, end, profiled = resolve_effective_speed(cmd, "driveDist")

    cfg = load_profile_config()
    # Signed cruise carries the power sign — NOT abs()-ed (the W1 bug).
    assert cruise == -70
    # Sign-by-cruise start seed follows the negative cruise into a reverse ramp.
    assert start == -cfg.start_speed
    assert start < 0
    # brake unset -> stop at rest (unaffected by the sign fix).
    assert end == 0
    assert profiled is True


def test_positive_power_still_resolves_forward_no_regression() -> None:
    """Contrast: a positive ``.setPower(70)`` still renders FORWARD.

    The signed-power fix must not regress the common forward case: a positive
    power resolves to a positive cruise and a forward (positive) start seed.
    """
    cmd = SimpleNamespace(
        power=70,
        preset=None,
        start_speed=None,
        end_speed=None,
        profiled=None,
        brake=None,
        reverse=False,
    )
    cruise, start, end, profiled = resolve_effective_speed(cmd, "driveDist")

    cfg = load_profile_config()
    assert cruise == 70
    assert start == cfg.start_speed
    assert start > 0
    assert end == 0
    assert profiled is True


def test_negative_power_wins_over_reverse_flag_no_double_negate() -> None:
    """When ``power`` is set, its sign is the direction — ``reverse`` doesn't double it.

    Direction comes from exactly one channel. A negative power with the
    ``reverse`` flag ALSO set must NOT flip back to forward (a double-negate);
    ``sign(power)`` wins, so the resolved cruise stays negative (reverse).
    """
    cmd = SimpleNamespace(
        power=-70,
        preset=None,
        start_speed=None,
        end_speed=None,
        profiled=None,
        brake=None,
        reverse=True,
    )
    cruise, start, _, _ = resolve_effective_speed(cmd, "driveDist")

    cfg = load_profile_config()
    assert cruise == -70
    assert start == -cfg.start_speed


# ---------------------------------------------------------------------------
# TC6 — display-only constant-rate + settle-allowance turn / pivot (AC7, T8)
# ---------------------------------------------------------------------------
#
# ADR-068 replaced the firmware's two-stage bang-bang turn executor with a
# per-tick PD controller whose true settling time is plant-dependent and has
# no closed-form solution (T8). The two-stage slow-zone-angle / slow-stage-PWM
# knobs it mirrored no longer exist (AC7). CTPS instead renders a NOMINAL
# model: a single constant angular rate derived from TURN_CRUISE_PWM (the
# PD's authority clamp), plus a fixed TURN_SETTLE_ALLOWANCE_S tail folded onto
# the final sub-keyframe only — a stand-in for the PD's creep-and-brake phase,
# not a second swept-angle stage (the render-layer-only posture ADR-061 took
# for HOP_SETTLE_MS).


def _expected_omega_deg_s(pwm: int, track_cm: float, is_pivot: bool) -> float:
    """Independently recompute the documented turn-rate formula for cross-checks.

    Mirrors :func:`tools.path_planner.ctps.profile._turn_omega`'s documented
    contract (a wheel at ``|pwm|`` moves ``SCALE_CM_S_PER_PWM * |pwm|`` cm/s; a
    counter-rotation spins both wheels; a pivot halves the rate) using only the
    PUBLIC :data:`SCALE_CM_S_PER_PWM` constant, so this test independently
    verifies the production formula rather than re-asserting its own output.
    """
    wheel_cm_s = SCALE_CM_S_PER_PWM * abs(pwm)
    omega = degrees(2.0 * wheel_cm_s / track_cm)
    return omega * 0.5 if is_pivot else omega


# A representative pure-heading in-place turn pose pair (90 deg CCW at a point).
_TURN_START_POSE = (10.0, 20.0, 0.0)
_TURN_END_POSE = (10.0, 20.0, 90.0)


def _sub_keyframe_rates(keyframes: list[dict]) -> list[float]:
    """Per-step angular rate (deg/s) between consecutive turn sub-keyframes.

    ``sample_turn`` emits keyframes evenly spaced in swept ANGLE, so the
    per-step rate = (angle swept in the step) / (time elapsed in the step). A
    constant-rate turn yields equal rates for every interval EXCEPT the last,
    which also carries the fixed settle-allowance tail and so renders with a
    strictly LOWER effective rate (same angle, more time).

    :param keyframes: an ordered turn keyframe list ``[{"t","x","y","h"}, ...]``.
    :return: one rate per interval; empty if fewer than two keyframes.
    """
    rates: list[float] = []
    for a, b in zip(keyframes, keyframes[1:]):
        dt = b["t"] - a["t"]
        d_ang = abs(((b["h"] - a["h"] + 180.0) % 360.0) - 180.0)
        if dt > 0.0:
            rates.append(d_ang / dt)
    return rates


@pytest.mark.parametrize("is_pivot", [False, True])
def test_tc6_default_config_turn_renders_constant_rate_with_settle_tail(
    is_pivot: bool,
) -> None:
    """TC6(a): with the default Config a turn/pivot renders CONSTANT-RATE.

    ``TURN_CRUISE_PWM = 50`` is read directly, unsnapped (the real firmware
    turn path no longer applies ``deadbandSnap`` to any turn command, AC3), and
    drives a SINGLE angular rate for the whole sweep. ``duration_s()`` is
    exactly the sweep time at that rate plus the fixed nominal settle
    allowance, and only the sampled sub-keyframes' FINAL interval carries that
    allowance (a longer last step into the target pose), never a second
    swept-angle stage.
    """
    cfg = load_profile_config()
    profile = resolve_turn_profile(90.0, is_pivot=is_pivot)

    assert profile.cruise_pwm == cfg.turn_cruise_pwm == 50
    assert profile.settle_s == pytest.approx(TURN_SETTLE_ALLOWANCE_S)

    expected_omega = _expected_omega_deg_s(cfg.turn_cruise_pwm, cfg.track_cm, is_pivot)
    assert profile.omega_deg_s == pytest.approx(expected_omega)

    # duration_s() == sweep time at the constant rate + the fixed settle tail.
    assert profile.duration_s() == pytest.approx(90.0 / profile.omega_deg_s + profile.settle_s)
    # time_at_angle at the full sweep agrees exactly with duration_s() (the
    # settle allowance is folded onto phi == |theta|, not bookkept separately).
    assert profile.time_at_angle(90.0) == pytest.approx(profile.duration_s())
    # Interior sub-sweeps never see the settle tail.
    assert profile.time_at_angle(45.0) == pytest.approx(45.0 / profile.omega_deg_s)

    # The sampled sub-keyframes: every interval but the last has the SAME rate;
    # the last is strictly slower (the settle allowance stretches it).
    kfs = sample_turn(profile, _TURN_START_POSE, _TURN_END_POSE, t0=0.0)
    rates = _sub_keyframe_rates(kfs)
    assert len(rates) >= 2
    for rate in rates[:-1]:
        assert rate == pytest.approx(rates[0], rel=1e-6)
    assert rates[-1] < rates[0]
    # The whole arc occupies exactly duration_s() seconds.
    assert kfs[-1]["t"] == pytest.approx(profile.duration_s())


def test_tc6_perturbed_cruise_pwm_scales_the_constant_rate(tmp_path: Path) -> None:
    """TC6(b): raising ``TURN_CRUISE_PWM`` scales the single constant rate.

    Via the ``config_path`` temp-header seam (the same seam TC2 uses), set
    ``TURN_CRUISE_PWM = 80``. The display-only model derives its single rate
    directly from this constant, so the rate — and the sweep-only portion of
    ``duration_s()`` — scale in exact (inverse) proportion to the PWM ratio.
    The fixed nominal settle allowance is untouched by Config (it is NOT a
    parsed constant, ADR-061 posture) and so is identical before and after.
    """
    perturbed_dir = tmp_path / "perturbed_turn_config"
    perturbed_dir.mkdir()
    perturbed_drive_h = _copy_config_headers(perturbed_dir)

    original_text = perturbed_drive_h.read_text(encoding="utf-8")
    lifted_text = original_text.replace(
        "constexpr int16_t TURN_CRUISE_PWM = TURN_CRUISE_PWM_DEFAULT;",
        "constexpr int16_t TURN_CRUISE_PWM = 80;",
    )
    assert lifted_text != original_text, (
        "the literal TURN_CRUISE_PWM declaration text did not match; "
        "ConfigDrive.h's declaration shape changed upstream"
    )
    perturbed_drive_h.write_text(lifted_text, encoding="utf-8")

    baseline_cfg = load_profile_config()
    perturbed_cfg = load_profile_config(perturbed_drive_h)
    assert baseline_cfg.turn_cruise_pwm == 50
    assert perturbed_cfg.turn_cruise_pwm == 80

    baseline = resolve_turn_profile(90.0, is_pivot=False, cfg=baseline_cfg)
    perturbed = resolve_turn_profile(90.0, is_pivot=False, cfg=perturbed_cfg)

    assert perturbed.cruise_pwm == 80
    assert perturbed.omega_deg_s == pytest.approx(baseline.omega_deg_s * 80.0 / 50.0)

    baseline_sweep_s = baseline.duration_s() - baseline.settle_s
    perturbed_sweep_s = perturbed.duration_s() - perturbed.settle_s
    assert perturbed_sweep_s == pytest.approx(baseline_sweep_s * 50.0 / 80.0)

    # The settle allowance is a fixed nominal display constant, not Config-derived.
    assert perturbed.settle_s == pytest.approx(baseline.settle_s)
    assert perturbed.settle_s == pytest.approx(TURN_SETTLE_ALLOWANCE_S)

    # The sampled sub-keyframes: still one constant rate + a strictly slower
    # final (settle-tailed) interval — never a second swept-angle stage.
    kfs = sample_turn(profile=perturbed, start_pose=_TURN_START_POSE, end_pose=_TURN_END_POSE, t0=0.0)
    rates = _sub_keyframe_rates(kfs)
    assert len(rates) >= 2
    for rate in rates[:-1]:
        assert rate == pytest.approx(rates[0], rel=1e-6)
    assert rates[-1] < rates[0]


# ---------------------------------------------------------------------------
# TC9 (ADR-059 / collinear-run, AC8) — LineFollower window-fit in the stop kernel
# ---------------------------------------------------------------------------
#
# The plan's AC8: a FAST (cruise 90) + REAR-led braked FINAL hop sheds
# `peak -> PWM_MIN` (the floor) over `(90 - 50) / accel = 8 cm`, but a REAR-led
# stop has only `LINE_SENSOR_AXLE_OFFSET_REAR_CM` (~7.68 cm) of overshoot window.
# The firmware fits the ramp-down to that window (`LineFollower.cpp:287-301`,
# `rampDownCm = min(default, window)` — a STEEPER shed that still reaches the
# floor before the brake), and `profile.py`'s stop kernel MODELS that fit rather
# than rendering an unfitted 8 cm floor-crawl tail the robot never drives. This
# is a RENDER requirement (AC8: "NOT a build-failing static_assert"), pinned here
# so a future refactor of `resolve_drive_profile` cannot silently re-break it.
#
# The rear window / front window / accel are SOURCED from the parsed config
# (`default_config().rear_axle_offset_cm`, not a literal — plan note), so a
# firmware retune of the offset stays consistent with this test.


#: A distance comfortably above `min_move_cm` and long enough for a full
#: trapezoid (a genuine cruise plateau + both ramps) at FAST cruise, so the
#: decel ramp is the natural `peak -> 0` shed the window-fit reshapes.
_AC8_STOP_DIST_CM: float = 40.0


def test_tc9_ac8_rear_window_fit_compresses_shed_to_window() -> None:
    """TC9 / AC8: a FAST + REAR-led final hop renders the window-fitted shed.

    ``resolve_drive_profile(dist, cruise=90 (FAST), start=35, end=0,
    profiled=True, rear_window_cm=rear_axle_offset_cm)`` must model the
    LineFollower window-fit: the natural ``peak -> floor`` shed against the
    KINETIC floor (``(90 - 30) / 4 = 15`` cm at cruise 90) OVERRUNS the ~7.68 cm
    rear window, so the display decel is compressed to the window — a steeper
    ramp that reaches the floor AT the distance gate.

    Pins the four AC8 post-conditions:

    * ``decel_cm <= rear_window`` — the shed fits inside the window,
    * ``floor_crossing_cm == dist`` — it reaches the floor exactly at the gate,
    * ``floor_tail_cm == 0`` — no floor-crawl tail past the gate,
    * ``abs(end_pwm) == PWM_MOVING_MIN`` — the display decel lands at the
      KINETIC decel-tail floor (not 0) at the gate, then brakes (the firmware
      ramp-down shape); the window-fit itself solves against the kinetic
      floor (``_window_fit_stop``'s ``floor_mag``), not the static one.
    """
    cfg = default_config()
    fast = cfg.speed_fast
    pwm_min = cfg.pwm_min
    moving_min = cfg.pwm_moving_min
    rear_window = float(cfg.rear_axle_offset_cm)

    # Precondition: FAST is the AC8 preset and the natural shed genuinely
    # overruns the rear window (else the fit would be a no-op and the test
    # vacuous). Natural above-floor shed = (|peak| - floor) / accel, mirroring
    # the production window-fit's floor operand — the KINETIC floor
    # (``_window_fit_stop`` is called with ``cfg.pwm_moving_min``), not the
    # static breakaway floor.
    assert fast == 90
    natural_shed = (fast - moving_min) / cfg.accel
    assert natural_shed > rear_window, (
        f"natural shed {natural_shed} cm must overrun the rear window "
        f"{rear_window} cm for this test to bind"
    )

    profile = resolve_drive_profile(
        _AC8_STOP_DIST_CM,
        cruise_pwm=fast,
        start_pwm=pwm_min,
        end_pwm=0,
        profiled=True,
        rear_window_cm=rear_window,
    )

    # The shed is compressed to the rear window and reaches the floor AT the gate.
    assert profile.decel_cm <= rear_window + 1e-6
    assert profile.decel_cm == pytest.approx(rear_window, abs=1e-6)
    assert profile.floor_crossing_cm == pytest.approx(profile.dist_cm, abs=1e-6)
    assert profile.floor_tail_cm == pytest.approx(0.0, abs=1e-6)
    # The display decel ramps peak -> floor (not to 0), so it lands at the
    # kinetic floor speed at the gate before the instantaneous brake.
    assert abs(profile.end_pwm) == moving_min


def test_tc9_ac8_front_window_binds_fast_shed() -> None:
    """TC9 / AC8: the FRONT window (12.02) NOW BINDS the FAST shed.

    The natural FAST shed against the KINETIC decel-tail floor is
    ``(speed_fast - pwm_moving_min) / accel = (90 - 30) / 4 = 15 cm``, which
    EXCEEDS the ~12.02 cm front overshoot window (``LINE_SENSOR_AXLE_OFFSET_CM``).
    So a front-led-window resolve FITS the ``peak -> floor`` shed to the window:
    the decel spans exactly the window, the floor-crossing pins to the gate, the
    crawl tail is dropped, and the display ``end_pwm`` becomes the sign-corrected
    KINETIC floor (the firmware ramps peak -> floor, not to 0) — the window-fit
    solves against ``cfg.pwm_moving_min``, not the static breakaway floor.
    """
    from tools.path_planner.config_parse import (
        DEFAULT_CONFIG_H_PATH,
        parse_config_float,
    )

    cfg = default_config()
    fast = cfg.speed_fast
    pwm_min = cfg.pwm_min
    moving_min = cfg.pwm_moving_min
    front_window = parse_config_float("LINE_SENSOR_AXLE_OFFSET_CM", DEFAULT_CONFIG_H_PATH)

    # Precondition: the natural FAST shed now OVERRUNS the front window, so the
    # window-fit must actually fire (the mirror image of the old no-op case).
    natural_shed = (fast - moving_min) / cfg.accel
    assert natural_shed > front_window, (
        f"natural shed {natural_shed} cm must exceed the front window "
        f"{front_window} cm for the window-fit to bind"
    )

    profile = resolve_drive_profile(
        _AC8_STOP_DIST_CM,
        cruise_pwm=fast,
        start_pwm=pwm_min,
        end_pwm=0,
        profiled=True,
        rear_window_cm=front_window,
    )

    # Binding: the shed is compressed to the window — the decel spans exactly the
    # window, the floor-crossing pins to the gate, the crawl tail is dropped, and
    # the display end_pwm ramps to the kinetic floor (not the 0-brake).
    assert profile.decel_cm == pytest.approx(front_window)
    assert profile.floor_crossing_cm == pytest.approx(_AC8_STOP_DIST_CM)
    assert profile.floor_tail_cm == pytest.approx(0.0)
    assert profile.end_pwm == moving_min

    # Cross-check: an unfitted resolve (rear_window_cm=None) keeps the NATURAL
    # stop — a floor-crawl tail survives and end_pwm stays the 0-brake. The window
    # genuinely reshaped the stop.
    natural = resolve_drive_profile(
        _AC8_STOP_DIST_CM,
        cruise_pwm=fast,
        start_pwm=pwm_min,
        end_pwm=0,
        profiled=True,
        rear_window_cm=None,
    )
    assert natural.floor_tail_cm > 0.0
    assert natural.end_pwm == 0
    assert profile.decel_cm != pytest.approx(natural.decel_cm)
    assert profile.floor_tail_cm != pytest.approx(natural.floor_tail_cm)


# ---------------------------------------------------------------------------
# TC11 — accel + cruise stay on the STATIC floor (phase-gating negative, AC7)
# ---------------------------------------------------------------------------
#
# The kinetic decel-tail floor (PWM_MOVING_MIN) must apply ONLY in the profile
# decel region (`velocity_at_s`'s `in_decel` gate). The accel ramp and the
# cruise plateau must keep floor-ing at the STATIC breakaway floor
# (PWM_MIN_MAGNITUDE), mirroring firmware `Drive::_decelTailFloorMag`. A move
# whose `start_pwm` AND `cruise_pwm` are both set BELOW the static floor is the
# only shape that can catch a kinetic-floor leak onto accel/cruise: if the
# implementation wrongly floored those phases at the kinetic floor (30) the
# displayed velocity would read low; the static floor (35) is the correct,
# higher value.


def test_tc11_accel_and_cruise_floor_at_static_not_kinetic_floor() -> None:
    """TC11: accel-ramp and cruise-plateau display velocity floors at ``pwm_min``.

    Builds a profiled move with ``start_pwm=10`` (below both floors) and
    ``cruise_pwm=20`` (also below both floors, so the cruise plateau's raw PWM
    magnitude is sub-floor and the floor clamp is actually exercised there too).
    Both the early-accel sample (raw ``pwm_at_s`` well under 30) and a
    mid-cruise sample must display at the STATIC floor's velocity
    (``SCALE_CM_S_PER_PWM * pwm_min``), never the lower kinetic-floor velocity
    (``SCALE_CM_S_PER_PWM * pwm_moving_min``) — guarding the phase gate in
    :meth:`DriveProfile.velocity_at_s`.
    """
    cfg = load_profile_config()
    assert cfg.pwm_moving_min < cfg.pwm_min, (
        "the kinetic floor must be strictly below the static floor for this "
        "test to distinguish the two"
    )

    profile = resolve_drive_profile(
        40.0, cruise_pwm=20, start_pwm=10, end_pwm=0, profiled=True, cfg=cfg
    )
    # Precondition: this shape has a genuine cruise plateau to sample (not a
    # collapsed triangle), and the raw peak/cruise PWM is itself sub-floor so
    # the floor clamp is exercised (not merely a no-op above-floor value).
    assert profile.cruise_cm > 0.0
    assert abs(profile.peak_pwm) < cfg.pwm_moving_min

    static_floor_v = SCALE_CM_S_PER_PWM * cfg.pwm_min
    kinetic_floor_v = SCALE_CM_S_PER_PWM * cfg.pwm_moving_min

    # Early accel-ramp sample: raw pwm_at_s is well below even the kinetic
    # floor, so the displayed velocity must be the STATIC floor, not the
    # (lower) kinetic floor.
    assert abs(profile.pwm_at_s(0.0)) < cfg.pwm_moving_min
    assert profile.velocity_at_s(0.0) == pytest.approx(static_floor_v)
    assert profile.velocity_at_s(0.0) != pytest.approx(kinetic_floor_v)

    # Mid-cruise sample: same guarantee on the plateau.
    s_cruise = profile.accel_cm + profile.cruise_cm / 2.0
    assert abs(profile.pwm_at_s(s_cruise)) < cfg.pwm_moving_min
    assert profile.velocity_at_s(s_cruise) == pytest.approx(static_floor_v)
    assert profile.velocity_at_s(s_cruise) != pytest.approx(kinetic_floor_v)


# ---------------------------------------------------------------------------
# TC13 — PWM_MOVING_MIN parses to 30 (AC1)
# ---------------------------------------------------------------------------


def test_tc13_pwm_moving_min_parses_to_30() -> None:
    """TC13: ``PWM_MOVING_MIN`` parses to 30 through the CTPS config loader.

    Mirrors how the sibling static floor (``PWM_MIN_MAGNITUDE`` / ``pwm_min``)
    is pinned elsewhere in this module (e.g. ``test_doubling_tshape_accel_...``
    asserts ``baseline_cfg.accel``): a plain sanity pin on
    ``load_profile_config()``'s parsed field, plus the raw
    :func:`parse_config_int` read against ``ConfigGlobal.h`` directly (the same
    two-layer shape ``test_config_parse.py`` uses for ``PWM_MIN_MAGNITUDE``),
    so a firmware retune of the seed value fails this test loudly rather than
    silently drifting from the render.
    """
    from tools.path_planner.config_parse import parse_config_int

    cfg = load_profile_config()
    assert cfg.pwm_moving_min == 30

    global_h = DEFAULT_CONFIG_H_PATH.parent / "ConfigGlobal.h"
    assert parse_config_int("PWM_MOVING_MIN", global_h) == 30
