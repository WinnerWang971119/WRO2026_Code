"""Firmware-faithful T-shape motion-profile kernel for CTPS playback (issue #130).

This module is the SINGLE Python source of the trapezoidal ("T-shape") velocity
profile math the CTPS player renders. It is pinned bit-for-bit to the firmware
by a cross-language golden vector test (T6/TC1): :func:`resolve_tshape_ramps`
is a pure mirror of ``TShapeProfilePolicy::resolveTShapeRamps``
(``WRO2026/src/drive/TShapeProfilePolicy.h``), and :func:`tshape_speed_at`
mirrors ``TShapeProfilePolicy::tShapeSpeedAt``.

Float32 emulation (the parity contract)
---------------------------------------
The C++ policy computes every real value in 32-bit ``float`` and casts the
interpolation results through ``int16_t`` (a truncation toward zero). A naive
``float64`` port diverges in the low bits and fails the golden-vector test. So
every real intermediate here rounds through :class:`numpy.float32` (via
:func:`_f32`), and every ``int16_t`` cast truncates toward zero via
:func:`_trunc_i16`. The triangle-peak divide is a truncating C-style ``int32 /
3`` (:func:`_trunc_div3`), which differs from Python's flooring ``//`` for a
negative sum.

The visual layer (closed-form phase durations, keyframe sampling, effective
speed resolution) is display-only and is NOT part of the golden-vector parity
contract — it is float64 for smoothness. Only the ramp/peak/speed-at kernel is
pinned.

The stop is modelled, not integrated to infinity
-------------------------------------------------
Naively clocking each keyframe with ``dt = ds / (k * pwm(s))`` diverges when the
end speed is 0 (the integral of ``ds/v`` near a linear zero is logarithmically
infinite). Instead the physical velocity floors at the motor deadband:
``v(s) = SCALE_CM_S_PER_PWM * max(|pwm(s)|, PWM_MIN_MAGNITUDE)``. Each ramp phase
gets a CLOSED-FORM duration (a log term above the floor, a linear term inside
the floor band). A profiled stop therefore renders as: decel ramp peak -> 50,
then a constant-50 floor-crawl to move end, then an INSTANTANEOUS brake — a
terminal keyframe that may share its predecessor's timestamp.

Public API (see the plan Contracts, 2026-07-09 CTPS motion-profile spec)
------------------------------------------------------------------------
- :func:`resolve_tshape_ramps` — pure float32 mirror of the C++ policy.
- :func:`tshape_speed_at` — pure float32 mirror of the C++ speed-at.
- :func:`resolve_drive_profile` -> :class:`DriveProfile` — Config-wired wrapper
  with phase boundaries, ``pwm_at_s``, closed-form phase durations.
- :func:`resolve_turn_profile` -> :class:`TurnProfile` — snapped per-stage PWM +
  derived angular rate.
- :func:`sample_drive` / :func:`sample_turn` — dense-near-ramps keyframe lists.
- :func:`resolve_effective_speed` — Config defaults + per-command overrides
  (sign-by-cruise start speed, no-brake -> endSpeed = cruise).
"""

from __future__ import annotations

from dataclasses import dataclass, field as dc_field
from math import ceil, degrees, log
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from tools.path_planner.config_parse import (
    DEFAULT_CONFIG_H_PATH,
    parse_config_float,
    parse_config_int,
)
from tools.path_planner.ctps.config import (
    ARC_STEP_DEG,
    KEYFRAME_COUNT_CEILING,
    RAMP_SAMPLE_STEP_CM,
    SCALE_CM_S_PER_PWM,
)

# ==============================================================================
# Float32 / int16 emulation primitives (the golden-vector parity contract)
# ==============================================================================


def _f32(x: float) -> float:
    """Round a real value to IEEE-754 binary32, returned as a Python float.

    Mirrors the C++ policy's 32-bit ``float`` arithmetic. Every real
    intermediate inside :func:`resolve_tshape_ramps` / :func:`tshape_speed_at`
    is passed through this so the Python result matches the host golden vectors
    bit-for-bit (:class:`numpy.float32` is the same binary32 the compiler emits).

    :param x: the value to narrow to float32.
    :return: ``x`` rounded to the nearest float32, widened back to a Python
        float (so downstream ``==`` compares exactly against a float32 golden).
    """
    return float(np.float32(x))


def _trunc_i16(x: float) -> int:
    """Truncate a real value toward zero, mirroring a C++ ``(int16_t)`` cast.

    C++ float->int conversion truncates toward zero (``(int16_t)-1.9f == -1``),
    which differs from Python's :func:`int` only in intent — :func:`int` already
    truncates toward zero for floats, so this is :func:`int` with an explicit
    name documenting the C++ semantic. No overflow wrap is modelled: every PWM
    in this domain is within ``[-PWM_MAX, PWM_MAX]`` (``|95|``), far inside the
    ``int16_t`` range, so a defensive mask would be dead code.

    :param x: the (float32) value being cast.
    :return: ``x`` truncated toward zero as a Python int.
    """
    return int(x)


def _trunc_div3(numerator: int) -> int:
    """C-style truncating integer divide by 3 (toward zero).

    Mirrors the C++ ``((int32_t)start + end + cruise) / 3`` in the triangle
    peak. C++ integer division truncates toward zero, so ``-7 / 3 == -2``,
    whereas Python's ``//`` FLOORS (``-7 // 3 == -3``). Using ``int(n / 3)``
    would round through a float and lose precision for large magnitudes; instead
    divide the integer magnitude and re-apply the sign, which is exact.

    :param numerator: the int32 sum ``start + end + cruise``.
    :return: ``numerator / 3`` truncated toward zero.
    """
    sign = -1 if numerator < 0 else 1
    return sign * (abs(numerator) // 3)


# ==============================================================================
# Config knobs (parsed once from the firmware headers via the T1 parser)
# ==============================================================================


@dataclass(frozen=True)
class ProfileConfig:
    """Parsed firmware Config constants the profile kernel needs.

    Read once from the ``WRO2026/config/*.h`` headers via the fail-loud
    :mod:`tools.path_planner.config_parse` parser (issue #44: no stale-literal
    fallback). Bundled into one immutable record so a caller resolves the
    headers a single time and threads the same values through every profile
    call — and so a test can inject a perturbed config (TC2) via
    :func:`load_profile_config` with a temp header path.

    All PWM values are signed magnitudes in the DriveDC ``[-100, 100]`` domain;
    all distances are cm; ``accel`` is PWM-per-cm.
    """

    accel: float          # TSHAPE_ACCEL_PWM_PER_CM
    ramp_min: float       # TSHAPE_RAMP_MIN_CM
    ramp_max: float       # TSHAPE_RAMP_MAX_CM
    no_ramp_eps: int      # TSHAPE_NO_RAMP_EPSILON_PWM
    pwm_min: int          # PWM_MIN_MAGNITUDE (motor deadband floor, 50)
    pwm_max: int          # PWM_MAX (saturation cap, 95)
    start_speed: int      # TSHAPE_START_SPEED (= PWM_MIN_MAGNITUDE)
    min_move_cm: float    # TSHAPE_MIN_MOVE_CM (5) — profiled/flat regime gate
    speed_slow: int       # SPEED_SLOW preset PWM
    speed_normal: int     # SPEED_NORMAL preset PWM (default cruise)
    speed_fast: int       # SPEED_FAST preset PWM
    line_cruise: int      # ConfigLine::CRUISE_SPEED (line-follow cruise)
    turn_cruise_pwm: int  # TURN_CRUISE_PWM
    turn_slow_pwm: int    # TURN_SLOW_SPEED_PWM
    wall_push_pwm: int     # WALL_STOP_PUSH_PWM
    wall_back_off_cm: float  # WALL_STOP_BACK_OFF_CM
    track_cm: float       # Config::TRACK_MM / 10 (chassis track width)


def load_profile_config(config_path: Path = DEFAULT_CONFIG_H_PATH) -> ProfileConfig:
    """Parse every Config constant the profile kernel needs, fail-loud.

    Each value is read from its OWNING firmware header via
    :mod:`tools.path_planner.config_parse` — the ONE place header parsing lives
    (issue #44). A missing/unparseable constant raises rather than falling back
    to a stale literal, surfacing at CTPS build time with the constant name +
    file (see the plan's Error Handling section).

    Constants are distributed across three headers:

    - ``ConfigDrive.h`` (``config_path``): the ``TSHAPE_*`` / ``TURN_*`` /
      ``WALL_STOP_*`` / ``SPEED_*`` / ``TRACK_MM`` knobs, plus the symbolic
      ``TSHAPE_START_SPEED = PWM_MIN_MAGNITUDE``.
    - ``ConfigGlobal.h``: ``PWM_MIN_MAGNITUDE`` / ``PWM_MAX``.
    - ``ConfigLine.h``: ``CRUISE_SPEED = SPEED_NORMAL`` (symbolic).

    The two sibling headers are resolved relative to ``config_path``'s directory
    so a test that copies a perturbed header set into a temp dir (TC2's
    ``config_path`` seam) reads a self-consistent trio, while the default reads
    the real firmware headers. The symbolic-RHS search set is likewise rooted at
    ``config_path``'s directory so ``TSHAPE_START_SPEED = PWM_MIN_MAGNITUDE`` and
    ``CRUISE_SPEED = SPEED_NORMAL`` resolve against the same (possibly temp)
    headers rather than the real ones.

    :param config_path: the ``ConfigDrive.h`` holding the drive constants;
        defaults to the real header resolved relative to this module
        (cwd-independent). Its parent directory must also hold ``ConfigGlobal.h``
        and ``ConfigLine.h``.
    :return: a fully-populated :class:`ProfileConfig`.
    :raises OSError: when ``config_path`` (or a sibling / symbolic-RHS search
        header) is unreadable.
    :raises ValueError: when a required constant is missing or unparseable.
    """
    from tools.path_planner.planner import parse_track_cm_from_config

    config_dir = config_path.parent
    global_h = config_dir / "ConfigGlobal.h"
    line_h = config_dir / "ConfigLine.h"
    # Symbolic RHS (TSHAPE_START_SPEED = PWM_MIN_MAGNITUDE, CRUISE_SPEED =
    # SPEED_NORMAL) resolves one level across the trio rooted at config_dir.
    search = (config_path, global_h, line_h)

    return ProfileConfig(
        accel=parse_config_float("TSHAPE_ACCEL_PWM_PER_CM", config_path),
        ramp_min=parse_config_float("TSHAPE_RAMP_MIN_CM", config_path),
        ramp_max=parse_config_float("TSHAPE_RAMP_MAX_CM", config_path),
        no_ramp_eps=parse_config_int("TSHAPE_NO_RAMP_EPSILON_PWM", config_path, search),
        pwm_min=parse_config_int("PWM_MIN_MAGNITUDE", global_h, search),
        pwm_max=parse_config_int("PWM_MAX", global_h, search),
        start_speed=parse_config_int("TSHAPE_START_SPEED", config_path, search),
        min_move_cm=parse_config_float("TSHAPE_MIN_MOVE_CM", config_path),
        speed_slow=parse_config_int("SPEED_SLOW", config_path, search),
        speed_normal=parse_config_int("SPEED_NORMAL", config_path, search),
        speed_fast=parse_config_int("SPEED_FAST", config_path, search),
        line_cruise=parse_config_int("CRUISE_SPEED", line_h, search),
        turn_cruise_pwm=parse_config_int("TURN_CRUISE_PWM", config_path, search),
        turn_slow_pwm=parse_config_int("TURN_SLOW_SPEED_PWM", config_path, search),
        wall_push_pwm=parse_config_int("WALL_STOP_PUSH_PWM", config_path, search),
        wall_back_off_cm=parse_config_float("WALL_STOP_BACK_OFF_CM", config_path),
        track_cm=parse_track_cm_from_config(config_path),
    )


# Module-level default config, parsed lazily on first use so importing this
# module never touches the filesystem (matches the deferred-import discipline
# in planner.py / validator.py). Callers that need a perturbed config pass an
# explicit ProfileConfig instead.
_DEFAULT_CONFIG: Optional[ProfileConfig] = None


def default_config() -> ProfileConfig:
    """Return the process-wide default :class:`ProfileConfig`, parsing it once.

    Lazily parses the real firmware headers on first call and caches the result.
    Keeps ``import tools.path_planner.ctps.profile`` filesystem-free (a test that
    only exercises :func:`resolve_tshape_ramps` with explicit args never triggers
    a header read). A perturbation test that needs different constants builds its
    own :class:`ProfileConfig` via :func:`load_profile_config` and does NOT go
    through this cache.

    :return: the cached default config, parsed from ``WRO2026/config/*.h``.
    """
    global _DEFAULT_CONFIG
    if _DEFAULT_CONFIG is None:
        _DEFAULT_CONFIG = load_profile_config()
    return _DEFAULT_CONFIG


# ==============================================================================
# Low-level ramp/peak/speed-at kernel — the golden-vector parity mirror
# ==============================================================================


def resolve_tshape_ramp_cm(
    cruise: int,
    endpoint: int,
    accel: float,
    ramp_min: float,
    ramp_max: float,
    no_ramp_eps: int,
) -> float:
    """Resolve ONE side's ramp distance (cm) — mirror of ``resolveTShapeRampCm``.

    Bit-for-bit mirror of the C++ ``resolveTShapeRampCm`` lambda: no-ramp epsilon
    first (strict ``<``, returns EXACTLY ``0.0`` — NOT clamped up to
    ``ramp_min``), then ``|cruise - endpoint| / accel`` in float32, clamped into
    ``[ramp_min, ramp_max]``.

    The magnitude is computed on the integer PWM delta (``int16_t`` promotes to
    int before the subtraction in C++), then divided by ``accel`` in float32.

    :param cruise: signed cruise PWM (sign = direction).
    :param endpoint: signed endpoint PWM (startSpeed or endSpeed) for this side.
    :param accel: ramp slope in PWM per cm (``TSHAPE_ACCEL_PWM_PER_CM``); > 0.
    :param ramp_min: ramp-distance floor (cm, ``TSHAPE_RAMP_MIN_CM``).
    :param ramp_max: ramp-distance ceiling (cm, ``TSHAPE_RAMP_MAX_CM``).
    :param no_ramp_eps: no-ramp hysteresis in PWM (``TSHAPE_NO_RAMP_EPSILON_PWM``).
    :return: ramp distance in cm; ``0.0`` when ``|cruise - endpoint| <
        no_ramp_eps`` (the no-ramp path).
    """
    # Integer delta magnitude — matches the C++ int subtraction before the cast.
    # Coerce to int up front: the firmware operands are always `int16_t`, so the
    # `deltaMag < noRampEps` compare is int-vs-int. A caller that passed a float
    # PWM would silently change the epsilon compare from the firmware's integer
    # semantics — coercing here keeps parity even for a hand-built test caller.
    delta = int(cruise) - int(endpoint)
    delta_mag = delta if delta > 0 else -delta
    if delta_mag < no_ramp_eps:
        return 0.0
    # float32 divide, then the two float32 clamps applied in order (min, max). The
    # ramp bounds are narrowed to float32 first so BOTH the comparison and the
    # assignment see the same float32 value the C++ `float rampMin`/`rampMax`
    # params hold (identical for the exact defaults 2.0/20.0; rigorous for any
    # non-exact retune).
    rmin = _f32(ramp_min)
    rmax = _f32(ramp_max)
    raw = _f32(_f32(float(delta_mag)) / _f32(accel))
    if raw < rmin:
        raw = rmin
    if raw > rmax:
        raw = rmax
    return raw


def resolve_tshape_ramps(
    dist_cm: float,
    cruise: int,
    start: int,
    end: int,
    accel: float,
    ramp_min: float,
    ramp_max: float,
    no_ramp_eps: int,
) -> Tuple[float, float, int]:
    """Resolve both ramp distances + the peak PWM — mirror of ``resolveTShapeRamps``.

    PURE float32 mirror of the C++ ``TShapeProfilePolicy::resolveTShapeRamps``.
    The signature matches the C++ 1:1 (``dist, cruise, start, end, accel``, then
    the ramp/eps knobs) so the golden-vector parity test (TC1) can call this
    directly with the goldens' own columns, no per-vector Config re-parse.

    The arithmetic is reproduced operation-for-operation (do NOT algebraically
    reassociate — the low bits must match the host dump):

    1. Defensive non-negative floor on ``dist_cm`` (mirrors ``_execProfileMove``).
    2. Per-side ramps via :func:`resolve_tshape_ramp_cm`.
    3. Triangle fallback on the UN-capped ramps (``accelCm + decelCm >
       totalCm``): peak = truncating ``(start + end + cruise) / 3`` cast to
       int16, then two sign-aware clamps toward cruise; then rescale
       ``scale = totalCm / (accelCm + decelCm)`` computed ONCE and each ramp
       multiplied by it (NOT reassociated).
    4. Half-of-total cap, AFTER the triangle test, against ``totalCm * 0.5f``.

    :param dist_cm: total travel (cm); negative floors to 0.
    :param cruise: signed cruise PWM (sign carries direction into the peak).
    :param start: signed PWM at move start.
    :param end: signed PWM at move end.
    :param accel: ramp slope in PWM per cm; > 0.
    :param ramp_min: per-side ramp floor (cm).
    :param ramp_max: per-side ramp ceiling (cm).
    :param no_ramp_eps: no-ramp hysteresis (PWM).
    :return: ``(accel_cm, decel_cm, peak_pwm)`` — per-side ramp distances (cm,
        each <= dist/2 after the cap) and the signed int16 peak PWM.
    """
    # Coerce the PWM operands to int up front — the firmware types are int16_t, so
    # the peak clamp/divide and the epsilon compare are integer arithmetic. This
    # keeps parity even if a golden-test caller hands in a numpy int / float column.
    cruise = int(cruise)
    start = int(start)
    end = int(end)

    total_cm = _f32(dist_cm)
    if total_cm < 0.0:
        total_cm = _f32(0.0)

    accel_cm = resolve_tshape_ramp_cm(cruise, start, accel, ramp_min, ramp_max, no_ramp_eps)
    decel_cm = resolve_tshape_ramp_cm(cruise, end, accel, ramp_min, ramp_max, no_ramp_eps)

    if _f32(accel_cm + decel_cm) > total_cm:
        # Triangle fallback: reduced averaged peak, clamped toward cruise.
        peak = _trunc_i16(_trunc_div3(int(start) + int(end) + int(cruise)))
        if peak > cruise and cruise >= 0:
            peak = cruise
        if peak < cruise and cruise < 0:
            peak = cruise
        peak_pwm = peak

        ramp_sum = _f32(accel_cm + decel_cm)
        if ramp_sum > 0.0:
            scale = _f32(total_cm / ramp_sum)
            accel_cm = _f32(accel_cm * scale)
            decel_cm = _f32(decel_cm * scale)
    else:
        peak_pwm = int(cruise)

    # Half-of-total cap. C++ re-evaluates `totalCm * 0.5f` at each of the four
    # use sites; hoisting it into `half` once is bit-identical ONLY because the
    # multiplier is exactly 0.5 — a power of two, so `x * 0.5f` is float32-exact
    # (it just decrements the exponent, never rounds) and deterministic. If the
    # firmware ever retunes this fraction to a non-power-of-two (e.g. 0.45f), a
    # per-use recompute could differ from this hoist in the low bit; then inline
    # `_f32(total_cm * _f32(<frac>))` at each site instead.
    half = _f32(total_cm * _f32(0.5))
    if accel_cm > half:
        accel_cm = half
    if decel_cm > half:
        decel_cm = half

    return accel_cm, decel_cm, peak_pwm


def tshape_speed_at(
    traveled_cm: float,
    total_cm: float,
    accel_cm: float,
    decel_cm: float,
    peak_pwm: int,
    start: int,
    end: int,
) -> int:
    """PWM setpoint at a traveled distance — mirror of ``tShapeSpeedAt``.

    PURE float32 mirror of the C++ ``TShapeProfilePolicy::tShapeSpeedAt``. The
    INNER cast order is load-bearing and reproduced exactly:
    ``(int16_t)(start + (int16_t)((peak - start) * ratio))`` — an int subtract,
    a float32 multiply, a TRUNCATING inner int16 cast, an int add of the
    endpoint, then the outer int16 cast.

    Branch predicates match the policy: ``traveled < accel`` (strict) is the
    ramp-up; ``total > decel and traveled >= total - decel`` is the ramp-down;
    otherwise hold at ``peak``. A zero-length ramp guard yields ``ratio = 1.0``.

    :param traveled_cm: distance traveled so far (same unit as ``total_cm``).
    :param total_cm: total move distance.
    :param accel_cm: resolved ramp-up distance from :func:`resolve_tshape_ramps`.
    :param decel_cm: resolved ramp-down distance.
    :param peak_pwm: signed peak PWM from :func:`resolve_tshape_ramps`.
    :param start: signed PWM at move start.
    :param end: signed PWM at move end.
    :return: signed int16 PWM setpoint (pre-``deadbandSnap``).
    """
    # Coerce PWM operands to int (firmware int16_t) so the int subtract / add and
    # the inner truncating cast are exact integer arithmetic for any caller.
    peak_pwm = int(peak_pwm)
    start = int(start)
    end = int(end)

    t = _f32(traveled_cm)
    total = _f32(total_cm)
    if t < accel_cm:
        ratio = _f32(1.0) if accel_cm == 0.0 else _f32(t / _f32(accel_cm))
        inner = _trunc_i16(_f32(_f32(peak_pwm - start) * ratio))
        return _trunc_i16(int(start) + inner)
    if total > decel_cm and t >= _f32(total - decel_cm):
        into = _f32(t - _f32(total - decel_cm))
        ratio = _f32(1.0) if decel_cm == 0.0 else _f32(into / _f32(decel_cm))
        inner = _trunc_i16(_f32(_f32(end - peak_pwm) * ratio))
        return _trunc_i16(int(peak_pwm) + inner)
    return int(peak_pwm)


# ==============================================================================
# deadbandSnap mirror (WROMath.h) — used for turn PWM and the velocity floor
# ==============================================================================


def deadband_snap(pwm: int, pwm_min: int, pwm_max: int) -> int:
    """Mirror of ``WROMath::deadbandSnap`` — floor a nonzero |PWM| up to the deadband.

    A signed PWM of 0 stays 0 (the caller meant to stop). Any nonzero magnitude
    below ``pwm_min`` is floored UP to ``pwm_min`` (the motor cannot act below
    its deadband), and any magnitude above ``pwm_max`` is clipped to ``pwm_max``,
    sign preserved. This is exactly the snap the firmware turn-emit path applies
    (``Drive.cpp:1173-1174``), which is why ``TURN_SLOW_SPEED_PWM = 20`` renders
    at ``|PWM| = 50``.

    :param pwm: signed PWM command.
    :param pwm_min: motor deadband floor (``PWM_MIN_MAGNITUDE``).
    :param pwm_max: saturation cap (``PWM_MAX``).
    :return: the snapped signed PWM.
    """
    if pwm == 0:
        return 0
    if pwm > 0:
        if pwm < pwm_min:
            return pwm_min
        if pwm > pwm_max:
            return pwm_max
        return pwm
    if pwm > -pwm_min:
        return -pwm_min
    if pwm < -pwm_max:
        return -pwm_max
    return pwm


# ==============================================================================
# Closed-form phase durations with the deadband velocity floor
# ==============================================================================
#
# Velocity model (display-only, float64):
#   v(s) = SCALE_CM_S_PER_PWM * max(|pwm(s)|, PWM_MIN_MAGNITUDE)
# where pwm(s) is LINEAR in distance along a ramp. The duration of a ramp is
# the closed-form integral of ds/v(s), split at the point where |pwm| crosses
# the deadband floor. This is what makes an endSpeed=0 stop FINITE (the naive
# dt = ds/(k*pwm) diverges near a linear zero).


def _linear_ramp_duration(
    length_cm: float,
    mag0: float,
    mag1: float,
    scale: float,
    floor: float,
) -> float:
    """Closed-form duration of a linear-|PWM| ramp over ``length_cm``, with a floor.

    The signed PWM ramps linearly in distance, so its MAGNITUDE ``m(s)`` is
    linear from ``mag0`` to ``mag1`` over ``[0, length_cm]``. Velocity is
    ``v(s) = scale * max(m(s), floor)``. This integrates ``ds / v(s)`` in closed
    form, splitting the interval at the distance where ``m(s)`` crosses
    ``floor``:

    - Above the floor, ``m`` linear: ``int ds/(scale*m)`` is ``L/(scale*m)`` when
      ``m`` is constant, else ``L/(scale*(m1-m0)) * ln(m1/m0)``.
    - Inside the floor band, ``v = scale*floor`` constant: ``Δs/(scale*floor)``.

    Handles both ramp directions (accelerating ``mag0 < mag1`` or decelerating
    ``mag0 > mag1``) and a fully-sub-floor ramp (pure floor-crawl). A zero-length
    ramp returns 0.

    :param length_cm: ramp distance (cm); 0 returns 0.
    :param mag0: |PWM| at the ramp start.
    :param mag1: |PWM| at the ramp end.
    :param scale: cm/s per PWM unit (``SCALE_CM_S_PER_PWM``); > 0.
    :param floor: deadband floor magnitude (``PWM_MIN_MAGNITUDE``); > 0.
    :return: non-negative ramp duration in seconds.
    """
    if length_cm <= 0.0:
        return 0.0

    # Both endpoints at or above the floor: single closed-form segment.
    if mag0 >= floor and mag1 >= floor:
        return _above_floor_segment(length_cm, mag0, mag1, scale)

    # Both endpoints at or below the floor: pure floor-crawl.
    if mag0 <= floor and mag1 <= floor:
        return length_cm / (scale * floor)

    # Mixed: exactly one endpoint crosses the floor. Split at the crossing
    # distance s_cross where m(s_cross) == floor. m is linear, so
    # s_cross / L = (floor - mag0) / (mag1 - mag0).
    frac = (floor - mag0) / (mag1 - mag0)
    s_cross = length_cm * frac
    above_len = length_cm - s_cross if mag1 > mag0 else s_cross
    below_len = length_cm - above_len

    # Above-floor part runs between whichever endpoint is >= floor and the floor.
    hi = max(mag0, mag1)
    above = _above_floor_segment(above_len, hi, floor, scale)
    below = below_len / (scale * floor)
    return above + below


def _above_floor_segment(length_cm: float, m0: float, m1: float, scale: float) -> float:
    """Duration of a linear-magnitude ramp fully at/above the floor (no clamping).

    Integrates ``ds / (scale * m(s))`` where ``m`` is linear from ``m0`` to
    ``m1`` over ``length_cm``. Constant magnitude collapses to ``L/(scale*m)``;
    otherwise the log form ``L/(scale*(m1-m0)) * ln(m1/m0)``. Both magnitudes are
    assumed strictly positive (the caller only routes at/above-floor segments
    here, and the floor is > 0).

    :param length_cm: segment distance (cm); 0 returns 0.
    :param m0: |PWM| at the segment start (> 0).
    :param m1: |PWM| at the segment end (> 0).
    :param scale: cm/s per PWM unit; > 0.
    :return: non-negative segment duration in seconds.
    """
    if length_cm <= 0.0:
        return 0.0
    if abs(m1 - m0) < 1e-9:
        return length_cm / (scale * m0)
    return length_cm / (scale * (m1 - m0)) * log(m1 / m0)


# ==============================================================================
# DriveProfile — Config-wired drive move (phase boundaries + durations)
# ==============================================================================


@dataclass(frozen=True)
class DriveProfile:
    """A resolved, Config-wired drive move: phase boundaries, PWM curve, durations.

    Produced by :func:`resolve_drive_profile`. Distances are along the move in
    cm; PWM is the signed pre-snap setpoint. Phase layout along ``[0, dist_cm]``:

        [0, accel_cm)                 ramp up   start_pwm -> peak_pwm
        [accel_cm, accel_cm+cruise_cm) cruise   hold at peak_pwm
        [.., floor_crossing_cm)       ramp down peak_pwm -> floor
        [floor_crossing_cm, dist_cm)  floor-crawl at PWM_MIN_MAGNITUDE
        dist_cm                       terminal brake (instantaneous)

    ``profiled=False`` (flat, or a ``< min_move_cm`` move) collapses to a single
    constant-PWM segment: ``accel_cm == decel_cm == 0``, ``peak_pwm == cruise``,
    the whole distance is "cruise", and there is no floor tail.
    """

    dist_cm: float
    cruise_pwm: int
    start_pwm: int
    end_pwm: int
    profiled: bool
    accel_cm: float
    cruise_cm: float
    decel_cm: float
    peak_pwm: int
    floor_crossing_cm: float  # distance at which the decel ramp reaches the floor
    floor_tail_cm: float      # dist_cm - floor_crossing_cm (constant-floor crawl)
    triangular: bool          # True iff the resolver's triangle-fallback branch fired
    _cfg: ProfileConfig = dc_field(repr=False)

    @property
    def regime(self) -> str:
        """Classify the move as ``"flat"`` / ``"triangular"`` / ``"trapezoid"``.

        Firmware-faithful (plan Decisions / TC3):

        - ``"flat"`` — the move degraded to constant cruise (``< min_move_cm`` or
          ``.flat()``): no ramps.
        - ``"triangular"`` — the resolver's triangle-fallback branch fired (the
          two full-peak ramps did not fit, so the plateau collapsed and the peak
          was reduced). This is :attr:`triangular`, set at resolve time — NOT
          ``cruise_cm == 0``, because the asymmetric half-of-total cap can trim
          one ramp and leave a small residual plateau at the reduced peak (see
          the ``TShapeProfilePolicy.h`` half-cap note); the branch still fired, so
          the move is triangular. The visible signature is ``peak_pwm`` magnitude
          below ``cruise_pwm`` magnitude.
        - ``"trapezoid"`` — the full-peak ramps fit; a genuine cruise plateau at
          ``cruise_pwm`` exists.

        :return: the regime label.
        """
        if not self.profiled:
            return "flat"
        return "triangular" if self.triangular else "trapezoid"

    def pwm_at_s(self, s: float) -> int:
        """Signed pre-snap PWM setpoint at distance ``s`` along the move.

        Clamps ``s`` into ``[0, dist_cm]`` then evaluates the firmware speed-at
        (:func:`tshape_speed_at`) for a profiled move, or returns the constant
        ``cruise_pwm`` for a flat move. This is the PWM curve the golden-pinned
        kernel produces — the display velocity floors it separately via
        :meth:`velocity_at_s`.

        :param s: distance along the move (cm).
        :return: signed int16 PWM setpoint (pre-``deadbandSnap``).
        """
        if not self.profiled:
            return int(self.cruise_pwm)
        s_clamped = 0.0 if s < 0.0 else (self.dist_cm if s > self.dist_cm else s)
        return tshape_speed_at(
            s_clamped,
            self.dist_cm,
            self.accel_cm,
            self.decel_cm,
            self.peak_pwm,
            self.start_pwm,
            self.end_pwm,
        )

    def velocity_at_s(self, s: float) -> float:
        """Display velocity (cm/s) at distance ``s`` — PWM floored at the deadband.

        ``v(s) = SCALE_CM_S_PER_PWM * max(|pwm_at_s(s)|, PWM_MIN_MAGNITUDE)``.
        The floor is what keeps a stop finite: at the end of a profiled move the
        PWM ramps to 0 but the chassis still crawls at ``PWM_MIN_MAGNITUDE`` until
        the terminal brake.

        Uses the int-truncated kernel PWM (:meth:`pwm_at_s`) so a displayed
        velocity readout matches the firmware setpoint at that point. Phase
        DURATIONS instead integrate the exact-float PWM
        (:meth:`_pwm_mag_exact_at_s`) so a keyframe's timestamp lands smoothly on
        the profile curve; the two agree to sub-PWM and are both display-only.

        :param s: distance along the move (cm).
        :return: strictly-positive display velocity in cm/s.
        """
        mag = abs(self.pwm_at_s(s))
        floored = max(mag, self._cfg.pwm_min)
        return SCALE_CM_S_PER_PWM * floored

    def _pwm_mag_exact_at_s(self, s: float) -> float:
        """Exact-float |PWM| magnitude at ``s`` for closed-form timing (no int cast).

        The display velocity model integrates a SMOOTH linear-in-distance PWM
        ramp — not the int-truncated firmware kernel — so the sum of the interior
        partial-phase durations equals the phase's full closed-form duration and a
        keyframe's timestamp sits exactly on the profile curve. Linear from
        ``|start_pwm|`` to ``|peak_pwm|`` on the accel ramp, constant ``|peak_pwm|``
        on cruise, linear from ``|peak_pwm|`` to ``|end_pwm|`` on the decel ramp.
        A flat move is constant ``|cruise_pwm|``. This magnitude is NOT floored —
        the caller applies the deadband floor.

        :param s: distance along the move (cm), clamped into ``[0, dist_cm]``.
        :return: the exact-float |PWM| magnitude (>= 0).
        """
        if not self.profiled:
            return abs(float(self.cruise_pwm))
        s_c = 0.0 if s < 0.0 else (self.dist_cm if s > self.dist_cm else s)
        if s_c < self.accel_cm and self.accel_cm > 0.0:
            frac = s_c / self.accel_cm
            return abs(float(self.start_pwm)) + (
                abs(float(self.peak_pwm)) - abs(float(self.start_pwm))
            ) * frac
        decel_start = self.dist_cm - self.decel_cm
        if self.decel_cm > 0.0 and s_c >= decel_start:
            frac = (s_c - decel_start) / self.decel_cm
            return abs(float(self.peak_pwm)) + (
                abs(float(self.end_pwm)) - abs(float(self.peak_pwm))
            ) * frac
        return abs(float(self.peak_pwm))

    def duration_s(self) -> float:
        """Total display duration of the move (seconds), closed-form.

        Sums the closed-form phase durations: ramp-up + cruise + ramp-down (with
        its floor-crossing split) + floor-crawl tail. The terminal brake is
        instantaneous (0 s). For a flat move this is the single constant-velocity
        segment ``dist / (scale * max(|cruise|, floor))``. A zero-length move is
        0 s (no divide-by-zero).

        :return: non-negative total duration in seconds.
        """
        return sum(seg[2] for seg in self._phase_segments())

    def _phase_segments(self) -> List[Tuple[float, float, float]]:
        """Return ``(s_start, s_end, duration_s)`` for each phase, in order.

        The building block for :meth:`duration_s` and :func:`sample_drive`:
        splits ``[0, dist_cm]`` into up to five spans (ramp-up, cruise, ramp-down
        above floor, floor-crawl tail) and computes each span's closed-form
        duration. Empty spans (zero length) are dropped. A flat move yields one
        constant-velocity span.

        :return: an ordered, gap-free list of ``(s_start, s_end, duration_s)``.
        """
        cfg = self._cfg
        scale = SCALE_CM_S_PER_PWM
        floor = float(cfg.pwm_min)

        if self.dist_cm <= 0.0:
            return []

        if not self.profiled:
            mag = max(abs(self.cruise_pwm), cfg.pwm_min)
            return [(0.0, self.dist_cm, self.dist_cm / (scale * mag))]

        segments: List[Tuple[float, float, float]] = []
        # Ramp-up: start_pwm -> peak_pwm over [0, accel_cm).
        a_end = self.accel_cm
        if a_end > 0.0:
            dur = _linear_ramp_duration(
                a_end, abs(self.start_pwm), abs(self.peak_pwm), scale, floor
            )
            segments.append((0.0, a_end, dur))
        # Cruise: constant peak over [accel_cm, accel_cm + cruise_cm).
        c_end = a_end + self.cruise_cm
        if self.cruise_cm > 0.0:
            mag = max(abs(self.peak_pwm), cfg.pwm_min)
            segments.append((a_end, c_end, self.cruise_cm / (scale * mag)))
        # Ramp-down above the floor: peak -> floor over [c_end, floor_crossing_cm).
        above_len = self.floor_crossing_cm - c_end
        if above_len > 0.0:
            dur = _linear_ramp_duration(
                above_len, abs(self.peak_pwm), floor, scale, floor
            )
            segments.append((c_end, self.floor_crossing_cm, dur))
        # Floor-crawl tail: constant floor over [floor_crossing_cm, dist_cm).
        if self.floor_tail_cm > 0.0:
            segments.append(
                (
                    self.floor_crossing_cm,
                    self.dist_cm,
                    self.floor_tail_cm / (scale * floor),
                )
            )
        return segments


def resolve_drive_profile(
    dist_cm: float,
    *,
    cruise_pwm: int,
    start_pwm: int,
    end_pwm: int,
    profiled: bool,
    cfg: Optional[ProfileConfig] = None,
) -> DriveProfile:
    """Resolve a Config-wired drive move into a :class:`DriveProfile`.

    Wraps :func:`resolve_tshape_ramps` with the parsed ``accel/ramp/eps`` knobs
    and derives the display phase boundaries. Regime selection is
    firmware-faithful (plan Decisions):

    - ``dist_cm < min_move_cm`` (5 cm) OR ``profiled=False`` -> **flat cruise**:
      one constant-``cruise_pwm`` segment, no ramps, no floor tail. (Drive gates
      the profiled move on ``_cm >= TSHAPE_MIN_MOVE_CM``; below that it enqueues a
      flat ``DRIVE_DIST``.)
    - triangle-branch fired (the two full-peak ramps did not fit) -> **triangular**:
      the plateau collapses and the peak is reduced. :attr:`DriveProfile.triangular`
      records the branch. NOTE: this is NOT the same as ``cruise_cm == 0`` — the
      asymmetric half-of-total cap can trim one ramp and leave a small residual
      plateau at the reduced peak (see the ``TShapeProfilePolicy.h`` half-cap
      note), so a triangular move may still report a small ``cruise_cm > 0``. The
      visible signature is ``|peak_pwm| < |cruise_pwm|``.
    - full-peak ramps fit -> **full trapezoid**: a genuine cruise plateau at
      ``cruise_pwm``.

    Degenerate inputs are floored defensively (mirroring ``_execProfileMove``):
    negative distance -> 0, ``accel <= 0`` clamps to a tiny positive slope so the
    kernel is well-defined (the firmware bails on ``accel <= 0`` before the math;
    the offline mirror must not divide by zero). A ``cruise_pwm`` of 0 yields a
    zero-velocity move whose display velocity still floors at the deadband.

    The floor-crossing distance is found by walking the decel ramp: it is where
    the pre-snap PWM magnitude first drops to ``pwm_min``. From that point to
    ``dist_cm`` the chassis crawls at the floor, then brakes.

    :param dist_cm: total travel distance (cm).
    :param cruise_pwm: signed cruise PWM (sign = direction).
    :param start_pwm: signed PWM at move start.
    :param end_pwm: signed PWM at move end.
    :param profiled: ``True`` for a ``.tShape()`` move, ``False`` for ``.flat()``.
    :param cfg: parsed Config; defaults to :func:`default_config`.
    :return: the resolved :class:`DriveProfile`.
    """
    cfg = cfg or default_config()

    dist = float(dist_cm)
    if dist < 0.0:
        dist = 0.0

    # A move below the profiled-gate threshold degrades to flat cruise, exactly
    # like the firmware (Drive.h gates PROFILE_DIST on _cm >= TSHAPE_MIN_MOVE_CM).
    is_flat = (not profiled) or dist < cfg.min_move_cm

    if is_flat:
        return DriveProfile(
            dist_cm=dist,
            cruise_pwm=int(cruise_pwm),
            start_pwm=int(start_pwm),
            end_pwm=int(end_pwm),
            profiled=False,
            accel_cm=0.0,
            cruise_cm=dist,
            decel_cm=0.0,
            peak_pwm=int(cruise_pwm),
            floor_crossing_cm=dist,
            floor_tail_cm=0.0,
            triangular=False,
            _cfg=cfg,
        )

    # Defensive accel floor: the firmware bails on accel <= 0 upstream, so this
    # branch is never reached with a bad accel in production; the offline mirror
    # clamps to a tiny positive slope so resolve_tshape_ramps never divides by
    # zero (Error Handling: mirror the defensive clamps).
    accel = cfg.accel if cfg.accel > 0.0 else 1e-6

    # Whether the triangle-fallback branch fires is decided on the UN-capped
    # per-side ramps (mirrors resolveTShapeRamps: the triangle test runs before
    # the half-of-total cap). Recompute the uncapped sum here purely to classify
    # the regime — resolve_tshape_ramps itself returns only the post-cap values.
    uncapped_accel = resolve_tshape_ramp_cm(
        int(cruise_pwm), int(start_pwm), accel, cfg.ramp_min, cfg.ramp_max, cfg.no_ramp_eps
    )
    uncapped_decel = resolve_tshape_ramp_cm(
        int(cruise_pwm), int(end_pwm), accel, cfg.ramp_min, cfg.ramp_max, cfg.no_ramp_eps
    )
    triangular = _f32(uncapped_accel + uncapped_decel) > _f32(dist)

    accel_cm, decel_cm, peak_pwm = resolve_tshape_ramps(
        dist,
        int(cruise_pwm),
        int(start_pwm),
        int(end_pwm),
        accel,
        cfg.ramp_min,
        cfg.ramp_max,
        cfg.no_ramp_eps,
    )
    cruise_cm = dist - accel_cm - decel_cm
    if cruise_cm < 0.0:
        cruise_cm = 0.0

    floor_crossing_cm = _decel_floor_crossing(
        dist, accel_cm, decel_cm, peak_pwm, start_pwm, end_pwm, cfg.pwm_min
    )
    floor_tail_cm = dist - floor_crossing_cm
    if floor_tail_cm < 0.0:
        floor_tail_cm = 0.0

    return DriveProfile(
        dist_cm=dist,
        cruise_pwm=int(cruise_pwm),
        start_pwm=int(start_pwm),
        end_pwm=int(end_pwm),
        profiled=True,
        accel_cm=accel_cm,
        cruise_cm=cruise_cm,
        decel_cm=decel_cm,
        peak_pwm=peak_pwm,
        floor_crossing_cm=floor_crossing_cm,
        floor_tail_cm=floor_tail_cm,
        triangular=triangular,
        _cfg=cfg,
    )


def _decel_floor_crossing(
    dist_cm: float,
    accel_cm: float,
    decel_cm: float,
    peak_pwm: int,
    start_pwm: int,
    end_pwm: int,
    pwm_min: int,
) -> float:
    """Distance at which the decel ramp's |PWM| first drops to the deadband floor.

    From this distance to ``dist_cm`` the chassis crawls at ``pwm_min`` (the
    floor-crawl tail) before the terminal brake. If the move never rises above /
    never drops to the floor on the decel side (e.g. ``end_pwm`` magnitude is
    already >= floor, so there is no sub-floor tail), the crossing is ``dist_cm``
    (no tail).

    The decel ramp is linear in distance from ``peak_pwm`` at
    ``dist_cm - decel_cm`` to ``end_pwm`` at ``dist_cm``. Solve for where its
    magnitude equals ``pwm_min``. Uses the display (float64) linear PWM, not the
    int-truncated :func:`tshape_speed_at`, so the crossing is smooth; the tiny
    difference from the int-cast curve is display-only and sub-pixel.

    :param dist_cm: total move distance (cm).
    :param accel_cm: resolved ramp-up distance.
    :param decel_cm: resolved ramp-down distance.
    :param peak_pwm: signed peak PWM.
    :param start_pwm: signed start PWM.
    :param end_pwm: signed end PWM.
    :param pwm_min: deadband floor magnitude.
    :return: the floor-crossing distance in ``[0, dist_cm]``.
    """
    # No decel ramp (chained no-ramp move): the crawl tail exists only if the
    # end magnitude is already below the floor across the whole move; but with no
    # ramp the PWM is constant at peak, so there is no descent — no tail.
    if decel_cm <= 0.0:
        return dist_cm

    peak_mag = abs(peak_pwm)
    end_mag = abs(end_pwm)

    # The decel ramp descends peak_mag -> end_mag over the last decel_cm.
    if end_mag >= pwm_min:
        # Never drops below the floor — no sub-floor crawl tail.
        return dist_cm
    if peak_mag <= pwm_min:
        # Whole decel ramp is already at/below the floor: the crawl tail begins
        # at the start of the decel ramp.
        return dist_cm - decel_cm

    # Linear descent from peak_mag to end_mag; find the fraction into the ramp
    # where the magnitude hits pwm_min.
    frac = (peak_mag - pwm_min) / (peak_mag - end_mag)
    return (dist_cm - decel_cm) + frac * decel_cm


# ==============================================================================
# TurnProfile — Config-wired turn / pivot (snapped per-stage PWM + rate)
# ==============================================================================


@dataclass(frozen=True)
class TurnProfile:
    """A resolved turn or pivot: snapped per-stage PWM + derived angular rate.

    Produced by :func:`resolve_turn_profile`. ``theta_deg`` is the signed swing
    (CCW positive). ``fast_pwm`` / ``slow_pwm`` are the deadband-SNAPPED per-stage
    magnitudes the firmware actually commands (``TURN_SLOW_SPEED_PWM = 20`` snaps
    to ``50``), so with the current Config both stages command ``|PWM| = 50`` and
    the render is CONSTANT-RATE. ``omega_deg_s`` is the derived angular rate at
    the snapped cruise stage (halved for a pivot — one wheel drives).

    A genuine two-stage ramp only appears when a cruise override exceeds the
    snapped floor; turn-override capture is deferred, so v1 always renders
    ``fast_pwm == slow_pwm == 50`` -> constant-rate.
    """

    theta_deg: float
    is_pivot: bool
    fast_pwm: int
    slow_pwm: int
    omega_deg_s: float
    _cfg: ProfileConfig = dc_field(repr=False)

    def duration_s(self) -> float:
        """Total display duration of the turn (seconds).

        Constant-rate at ``omega_deg_s``: ``|theta| / omega``. A zero-angle turn
        (or a degenerate zero rate) is 0 s (no divide-by-zero — the ``omega``
        guard is in :func:`resolve_turn_profile`).

        :return: non-negative turn duration in seconds.
        """
        if self.omega_deg_s <= 0.0:
            return 0.0
        return abs(self.theta_deg) / self.omega_deg_s


def resolve_turn_profile(
    theta_deg: float,
    *,
    is_pivot: bool,
    cruise_pwm: Optional[int] = None,
    slow_pwm: Optional[int] = None,
    cfg: Optional[ProfileConfig] = None,
) -> TurnProfile:
    """Resolve a turn / pivot into a :class:`TurnProfile` with snapped PWM + rate.

    Per the plan Decisions (AC3), the per-stage PWM is the deadband-SNAPPED
    command the firmware emits, NOT the raw ``TURN_*`` Config value. With the
    default Config (``TURN_CRUISE_PWM = 50``, ``TURN_SLOW_SPEED_PWM = 20``,
    ``PWM_MIN_MAGNITUDE = 50``) both stages snap to ``|PWM| = 50`` -> a
    constant-rate arc. A two-stage ramp appears only if a cruise override > 50 is
    supplied (deferred capture — v1 leaves ``cruise_pwm`` None).

    The angular rate is DERIVED from the single visual scale + the chassis track
    (no second scale constant): a wheel commanded at PWM ``p`` moves at
    ``v = SCALE * p`` cm/s; a counter-rotation spins both wheels, so
    ``omega = (2 * v / TRACK_CM) * 180/pi``; a pivot drives ONE wheel about the
    other, HALVING the rate.

    :param theta_deg: signed swing angle (deg, CCW positive).
    :param is_pivot: ``True`` for a single-wheel pivot (rate halved).
    :param cruise_pwm: fast-stage PWM override; ``None`` uses ``TURN_CRUISE_PWM``.
    :param slow_pwm: slow-stage PWM override; ``None`` uses ``TURN_SLOW_SPEED_PWM``.
    :param cfg: parsed Config; defaults to :func:`default_config`.
    :return: the resolved :class:`TurnProfile`.
    """
    cfg = cfg or default_config()

    raw_fast = cfg.turn_cruise_pwm if cruise_pwm is None else int(cruise_pwm)
    raw_slow = cfg.turn_slow_pwm if slow_pwm is None else int(slow_pwm)

    fast_pwm = deadband_snap(raw_fast, cfg.pwm_min, cfg.pwm_max)
    slow_pwm = deadband_snap(raw_slow, cfg.pwm_min, cfg.pwm_max)

    # Angular rate at the (snapped) fast stage. The magnitude drives the arc; the
    # sign of theta carries the direction separately.
    wheel_cm_s = SCALE_CM_S_PER_PWM * abs(fast_pwm)
    # omega = (2*v / track) rad/s -> deg/s; pivot drives one wheel -> half.
    omega_deg_s = degrees(2.0 * wheel_cm_s / cfg.track_cm)
    if is_pivot:
        omega_deg_s *= 0.5

    return TurnProfile(
        theta_deg=float(theta_deg),
        is_pivot=is_pivot,
        fast_pwm=fast_pwm,
        slow_pwm=slow_pwm,
        omega_deg_s=omega_deg_s,
        _cfg=cfg,
    )


# ==============================================================================
# Keyframe sampling — dense near ramps, sparse on cruise
# ==============================================================================


def sample_drive(
    profile: DriveProfile,
    start_pose: Tuple[float, float, float],
    end_pose: Tuple[float, float, float],
    t0: float,
) -> List[Dict[str, Any]]:
    """Sample a drive move into dense keyframes whose timestamps follow the profile.

    Emits keyframes at every phase boundary plus interior ramp samples spaced no
    more than ``RAMP_SAMPLE_STEP_CM`` apart (cruise / floor-crawl spans get only
    their boundary keyframes — PWM is constant there, so denser samples add no
    visual information). Each keyframe's pose is a LINEAR interpolation of
    ``start_pose -> end_pose`` by traveled-distance fraction (x/y lerp, heading
    on the shortest arc); its timestamp is ``t0 + closed-form-duration-so-far``.

    The stop is visible: the decel ramp emits samples down to the floor crossing,
    then the floor-crawl tail is a boundary keyframe, then a TERMINAL keyframe at
    ``end_pose`` representing the instantaneous brake — it may share the previous
    keyframe's timestamp (the caller/player tolerates a zero-length bracket).

    The endpoints are forced EXACTLY to ``start_pose`` / ``end_pose`` (byte-equal)
    so T10 can pin the last sub-keyframe to the sim snapshot pose. The keyframe
    count is capped at ``KEYFRAME_COUNT_CEILING`` — interior sampling coarsens
    rather than exceeding it.

    Timestamps are NON-DECREASING; a zero-length move emits a single keyframe at
    ``start_pose`` (which equals ``end_pose``), with no divide-by-zero.

    :param profile: the resolved :class:`DriveProfile`.
    :param start_pose: ``(x, y, heading_deg)`` at ``s = 0``.
    :param end_pose: ``(x, y, heading_deg)`` at ``s = dist_cm``.
    :param t0: timestamp of the first sub-keyframe (the arc-start clock).
    :return: an ordered keyframe list ``[{"t", "x", "y", "h"}, ...]`` whose first
        pose is ``start_pose`` and whose last pose is ``end_pose``.
    """
    x0, y0, h0 = start_pose
    x1, y1, h1 = end_pose
    dh = _shortest_arc(h0, h1)

    dist = profile.dist_cm
    if dist <= 0.0:
        # Zero-length segment: a single keyframe (no duration, no divide-by-zero).
        return [_pose_kf(t0, x0, y0, h0)]

    # Build the ordered set of sample distances: phase boundaries + interior ramp
    # samples, deduplicated and clamped to [0, dist].
    distances = _drive_sample_distances(profile)

    # Map each sample distance to its cumulative closed-form time by walking the
    # phase segments (each carries its own closed-form duration).
    time_at = _cumulative_time_fn(profile)

    keyframes: List[Dict[str, Any]] = []
    for s in distances:
        frac = s / dist
        x = x0 + (x1 - x0) * frac
        y = y0 + (y1 - y0) * frac
        h = (h0 + dh * frac) % 360.0
        keyframes.append(_pose_kf(t0 + time_at(s), x, y, h))

    # Force the endpoints byte-exact (guards float lerp drift at s == dist and
    # lets T10 pin the last sub-keyframe to the snapshot pose).
    keyframes[0] = _pose_kf(keyframes[0]["t"], x0, y0, h0)
    keyframes[-1] = _pose_kf(keyframes[-1]["t"], x1, y1, h1)

    # Terminal instantaneous brake: for a profiled stop the last boundary is the
    # floor-crawl end == end_pose already at the total duration, so no extra
    # keyframe is needed — the pose keyframe at s == dist IS the brake point. The
    # non-decreasing-timestamp contract holds because time_at is monotonic.
    return _enforce_non_decreasing(keyframes)


def _drive_sample_distances(profile: DriveProfile) -> List[float]:
    """Ordered, deduplicated sample distances for a drive move.

    Phase boundaries (0, accel end, cruise end, floor crossing, dist) plus
    interior ramp samples spaced <= ``RAMP_SAMPLE_STEP_CM`` on the accel and
    (above-floor) decel ramps only. Cruise and floor-crawl spans contribute only
    their boundaries. The total sample count is capped at
    ``KEYFRAME_COUNT_CEILING`` by widening the ramp step if needed.

    :param profile: the resolved drive profile.
    :return: strictly-increasing sample distances covering ``[0, dist_cm]``.
    """
    dist = profile.dist_cm
    boundaries = {0.0, dist}

    if profile.profiled:
        a_end = profile.accel_cm
        c_end = a_end + profile.cruise_cm
        boundaries.update({a_end, c_end, profile.floor_crossing_cm})

        # Interior ramp samples. Budget the step so total count stays under the
        # ceiling (leave headroom for the ~4 boundary points).
        ramp_span = a_end + (profile.floor_crossing_cm - c_end)
        step = RAMP_SAMPLE_STEP_CM
        if ramp_span > 0.0:
            max_interior = max(1, KEYFRAME_COUNT_CEILING - len(boundaries) - 1)
            needed = ceil(ramp_span / step)
            if needed > max_interior:
                step = ramp_span / max_interior

        _add_interior_samples(boundaries, 0.0, a_end, step)
        _add_interior_samples(boundaries, c_end, profile.floor_crossing_cm, step)

    ordered = sorted(b for b in boundaries if 0.0 <= b <= dist)
    return ordered


def _add_interior_samples(
    acc: set, lo: float, hi: float, step: float
) -> None:
    """Add interior sample distances strictly inside ``(lo, hi)`` at ``step`` spacing.

    Mutates ``acc`` (a set of distances) in place. Endpoints are NOT added here —
    they are already phase boundaries. A non-positive span or step is a no-op.

    :param acc: the running set of sample distances (mutated).
    :param lo: span start (exclusive).
    :param hi: span end (exclusive).
    :param step: maximum spacing between interior samples (cm).
    """
    if hi - lo <= step or step <= 0.0:
        return
    n = ceil((hi - lo) / step)
    for k in range(1, n):
        acc.add(lo + (hi - lo) * (k / n))


def _cumulative_time_fn(profile: DriveProfile):
    """Return a monotonic ``s -> cumulative closed-form time`` function.

    Precomputes the phase segments (each with its own closed-form duration) and
    returns a closure that, for a distance ``s``, sums complete prior phases plus
    the partial closed-form duration of the phase ``s`` falls in. This is what
    makes the keyframe timestamps follow the PROFILE (dense in time near the
    ramps, where velocity is low) rather than a uniform clock.

    :param profile: the resolved drive profile.
    :return: a callable ``time_at(s) -> float`` non-decreasing over ``[0, dist]``.
    """
    segments = profile._phase_segments()
    cfg = profile._cfg
    scale = SCALE_CM_S_PER_PWM
    floor = float(cfg.pwm_min)

    # Precompute the cumulative time at the START of each segment.
    starts: List[Tuple[float, float, float, float]] = []  # (s0, s1, t0_seg, dur)
    t_acc = 0.0
    for (s0, s1, dur) in segments:
        starts.append((s0, s1, t_acc, dur))
        t_acc += dur
    total_t = t_acc

    def time_at(s: float) -> float:
        if s <= 0.0:
            return 0.0
        if s >= profile.dist_cm:
            return total_t
        for (s0, s1, t0_seg, dur) in starts:
            if s <= s1 or (s1 == s0):
                if s <= s0:
                    return t0_seg
                # Partial closed-form duration from s0 to s within this phase.
                return t0_seg + _partial_phase_duration(
                    profile, s0, s1, s, scale, floor
                )
        return total_t

    return time_at


def _partial_phase_duration(
    profile: DriveProfile,
    s0: float,
    s1: float,
    s: float,
    scale: float,
    floor: float,
) -> float:
    """Closed-form duration from ``s0`` to ``s`` within one phase span ``[s0, s1]``.

    Re-derives the phase's velocity endpoints (the |PWM| at ``s0`` and ``s``) and
    integrates ``ds/v`` over ``[s0, s]`` with the same floor-aware closed form as
    the full-phase duration. Used by :func:`_cumulative_time_fn` to place an
    interior keyframe's timestamp exactly on the profile curve. A zero-width span
    returns 0.

    :param profile: the resolved drive profile (for the PWM curve).
    :param s0: phase start distance.
    :param s1: phase end distance.
    :param s: the interior distance (``s0 <= s <= s1``).
    :param scale: cm/s per PWM unit.
    :param floor: deadband floor magnitude.
    :return: non-negative partial duration in seconds.
    """
    if s <= s0 or s1 <= s0:
        return 0.0
    # Use the exact-float PWM (not the int-truncated kernel) so the sum of the
    # partial durations across a phase equals that phase's full closed-form
    # duration — interior keyframe timestamps then land exactly on the profile
    # curve. The floor split inside _linear_ramp_duration handles a partial span
    # that crosses the deadband on the decel ramp.
    mag0 = profile._pwm_mag_exact_at_s(s0)
    mag_s = profile._pwm_mag_exact_at_s(s)
    return _linear_ramp_duration(s - s0, mag0, mag_s, scale, floor)


def sample_turn(
    profile: TurnProfile,
    start_pose: Tuple[float, float, float],
    end_pose: Tuple[float, float, float],
    t0: float,
    arc_keyframes: Optional[List[Dict[str, Any]]] = None,
) -> List[Dict[str, Any]]:
    """Sample a turn / pivot into evenly-timed keyframes at the snapped rate.

    Two modes:

    1. ``arc_keyframes`` given (the pivot-arc path): T10 has already computed the
       geometric arc sub-keyframes (via ``_build_pivot_arc``); this RE-TIMES them
       so the whole arc occupies ``profile.duration_s()`` at the snapped
       constant rate. Timestamps are pinned to ``t0 + k/N * duration`` (never
       accumulated) so the clock is exactly monotonic and the arc poses are left
       untouched. The final sub-keyframe's pose is preserved byte-for-byte.
    2. ``arc_keyframes`` None (an in-place turn): emits evenly-spaced keyframes
       interpolating ``start_pose -> end_pose`` (position lerp — usually a pure
       heading change so x/y are equal — heading on the signed swing) with
       constant-rate timestamps. Sample count tracks ``ARC``-style density but is
       capped at ``KEYFRAME_COUNT_CEILING``.

    Constant-rate (both Config stages snap to ``|PWM| = 50``) means EVENLY-timed
    sub-keyframes — the tripwire TC6 asserts. The endpoints are forced exact so
    T10 can pin the last sub-keyframe to the snapshot pose.

    :param profile: the resolved :class:`TurnProfile`.
    :param start_pose: ``(x, y, heading_deg)`` at the turn start.
    :param end_pose: ``(x, y, heading_deg)`` at the turn end.
    :param t0: timestamp of the turn-start keyframe.
    :param arc_keyframes: pre-computed pivot-arc sub-keyframes to re-time, or
        ``None`` to synthesize an in-place turn.
    :return: an ordered, evenly-timed keyframe list.
    """
    duration = profile.duration_s()

    if arc_keyframes is not None:
        n = len(arc_keyframes)
        if n == 0:
            return []
        retimed: List[Dict[str, Any]] = []
        for k, kf in enumerate(arc_keyframes, start=1):
            t = t0 + (duration * k / n)
            retimed.append(_pose_kf(t, kf["x"], kf["y"], kf["h"]))
        return _enforce_non_decreasing(retimed)

    # In-place turn: evenly-spaced constant-rate keyframes start -> end.
    x0, y0, h0 = start_pose
    x1, y1, h1 = end_pose
    dh = _shortest_arc(h0, h1)

    abs_theta = abs(profile.theta_deg) if profile.theta_deg else abs(dh)
    n = max(1, ceil(abs_theta / ARC_STEP_DEG))
    n = min(n, KEYFRAME_COUNT_CEILING - 1)
    n = max(1, n)

    dt = duration / n if n > 0 else 0.0
    keyframes = [_pose_kf(t0, x0, y0, h0)]
    for k in range(1, n + 1):
        frac = k / n
        x = x0 + (x1 - x0) * frac
        y = y0 + (y1 - y0) * frac
        h = (h0 + dh * frac) % 360.0
        keyframes.append(_pose_kf(t0 + dt * k, x, y, h))
    keyframes[-1] = _pose_kf(keyframes[-1]["t"], x1, y1, h1)
    return _enforce_non_decreasing(keyframes)


# ==============================================================================
# Effective-speed resolution — Config defaults + per-command overrides
# ==============================================================================


def resolve_effective_speed(cmd: Any, method: str) -> Tuple[int, int, int, bool]:
    """Resolve a recorded drive command's effective ``(cruise, start, end, profiled)``.

    Applies the firmware default rules to a recorded ``drive`` op (the additive
    optional fields ``power/preset/start_speed/end_speed/profiled/brake`` from the
    Contracts JSON seam; absent -> Config default). ``cmd`` is duck-typed: any
    object exposing those attributes (a loaded ``Command``, a namespace, a dict
    wrapped by :func:`_getattr`). Rules (plan Decisions):

    - **cruise**: ``power`` overrides everything; else the ``preset`` maps
      (slow/normal/fast) — default NORMAL. For a LINE-follow method the default
      cruise is ``ConfigLine::CRUISE_SPEED`` instead of the Drive preset.
    - **direction sign**: a ``reverse`` command negates the cruise magnitude so a
      reverse hop drives a reverse profile.
    - **start**: default is SIGN-BY-CRUISE —
      ``cruise >= 0 ? +TSHAPE_START_SPEED : -TSHAPE_START_SPEED`` — so a reverse
      hop seeds a REVERSE ramp, not a forward one. An explicit ``start_speed``
      overrides.
    - **end**: default is 0 (brake to rest); ``.setBrake(false)`` (``brake ==
      False``) defaults ``end`` to ``cruise`` (NOT 0), so a no-brake / chained
      move renders no phantom decel-to-zero. An explicit ``end_speed`` overrides
      either default.
    - **profiled**: the recorded ``profiled`` flag (``.tShape()`` -> True,
      ``.flat()`` -> False); absent -> True (the ``driveDist`` builder default).

    :param cmd: the recorded drive command (duck-typed accessors, see above).
    :param method: the recorded method name (``"driveDist"``, ``"lineFollowDist"``,
        ``"stopAtLine"``, ``"wallStop"``, ...); selects the line-vs-drive cruise
        default.
    :return: ``(cruise, start, end, profiled)`` — signed PWM for the first three,
        bool for profiled.
    """
    cfg = default_config()

    power = _getattr(cmd, "power")
    preset = _getattr(cmd, "preset")
    start_speed = _getattr(cmd, "start_speed")
    end_speed = _getattr(cmd, "end_speed")
    profiled_flag = _getattr(cmd, "profiled")
    brake = _getattr(cmd, "brake")
    reverse = bool(_getattr(cmd, "reverse") or False)

    # Only the line-FOLLOW method defaults its cruise to ConfigLine::CRUISE_SPEED.
    # `stopAtLine` is a distance-gated SEEK, not a follow — it must NOT match here
    # (it defaults to the Drive preset), so this is an exact method-name check, not
    # a loose "line" substring (which would wrongly catch `stopAtLine`).
    is_line = method.strip().lower() == "linefollowdist"

    # --- cruise magnitude ---
    if power is not None:
        cruise_mag = abs(int(power))
    elif preset is not None:
        cruise_mag = _preset_to_pwm(str(preset), cfg)
    elif is_line:
        cruise_mag = int(cfg.line_cruise)
    else:
        cruise_mag = int(cfg.speed_normal)

    cruise = -cruise_mag if reverse else cruise_mag

    # --- start speed: sign-by-cruise default ---
    if start_speed is not None:
        start = int(start_speed)
    else:
        start = cfg.start_speed if cruise >= 0 else -cfg.start_speed

    # --- end speed: brake rule ---
    if end_speed is not None:
        end = int(end_speed)
    elif brake is False:
        # .setBrake(false) -> chain into a follow-up: default end = cruise.
        end = cruise
    else:
        # Default (brake true / unset) -> stop at rest.
        end = 0

    # --- profiled flag ---
    profiled = True if profiled_flag is None else bool(profiled_flag)

    return cruise, start, end, profiled


def _preset_to_pwm(preset: str, cfg: ProfileConfig) -> int:
    """Map a recorded preset name to its Config PWM magnitude.

    Mirrors ``WROMath::presetToPwm`` over the recorded string form (the recorder
    serializes the ``SpeedPreset`` enum as ``"slow"/"normal"/"fast"``). An
    unrecognized preset falls back to NORMAL rather than raising — the loader
    already strict-validates the field, so this is a defensive default, not a
    silent-corruption path.

    :param preset: the recorded preset name (case-insensitive).
    :param cfg: parsed Config carrying the preset PWM constants.
    :return: the preset's PWM magnitude.
    """
    key = preset.strip().lower()
    if key == "slow":
        return int(cfg.speed_slow)
    if key == "fast":
        return int(cfg.speed_fast)
    return int(cfg.speed_normal)


# ==============================================================================
# Small shared helpers
# ==============================================================================


def _getattr(cmd: Any, name: str) -> Any:
    """Read an optional attribute from a duck-typed command, ``None`` if absent.

    Supports both attribute-style (a ``Command`` dataclass / namespace) and
    mapping-style (a plain dict) commands so :func:`resolve_effective_speed`
    works against a loaded trace op or a test stub uniformly.

    :param cmd: the command object or dict.
    :param name: the field name to read.
    :return: the value, or ``None`` when the field is absent.
    """
    if isinstance(cmd, dict):
        return cmd.get(name)
    return getattr(cmd, name, None)


def _pose_kf(t: float, x: float, y: float, h: float) -> Dict[str, Any]:
    """Build a keyframe dict in the CTPS ``{"t", "x", "y", "h"}`` shape.

    Matches the keyframe shape ``track_builder`` emits (heading is folded into
    ``[0, 360)`` there; the caller passes an already-folded heading for interior
    samples, and the endpoint override preserves the caller's exact pose).

    :param t: timestamp (seconds).
    :param x: chassis-centre x (cm).
    :param y: chassis-centre y (cm).
    :param h: heading (deg).
    :return: the keyframe dict.
    """
    return {"t": t, "x": x, "y": y, "h": h}


def _shortest_arc(h0: float, h1: float) -> float:
    """Signed shortest heading delta ``h0 -> h1`` in ``(-180, +180]``.

    So interior keyframes interpolate a turn along the shorter arc (a 350 -> 10
    move sweeps +20, not -340), matching the player's shortest-arc heading lerp.

    :param h0: start heading (deg).
    :param h1: end heading (deg).
    :return: signed delta in ``(-180, +180]``.
    """
    d = (h1 - h0) % 360.0
    if d > 180.0:
        d -= 360.0
    return d


def _enforce_non_decreasing(keyframes: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """Clamp keyframe timestamps to be non-decreasing (the AC10 contract).

    Float rounding in the closed-form durations can, in rare degenerate cases,
    produce a timestamp a hair below its predecessor; the player's bisect assumes
    a non-decreasing clock. This clamps each timestamp up to its predecessor's
    when it would otherwise dip (a terminal brake keyframe legitimately EQUALS
    its predecessor's ``t`` — that is allowed, only a strict DECREASE is fixed).
    Poses are untouched.

    :param keyframes: the ordered keyframe list (mutated in place and returned).
    :return: the same list with a non-decreasing ``t`` sequence.
    """
    for i in range(1, len(keyframes)):
        if keyframes[i]["t"] < keyframes[i - 1]["t"]:
            keyframes[i]["t"] = keyframes[i - 1]["t"]
    return keyframes


__all__ = [
    "ProfileConfig",
    "DriveProfile",
    "TurnProfile",
    "load_profile_config",
    "default_config",
    "resolve_tshape_ramp_cm",
    "resolve_tshape_ramps",
    "tshape_speed_at",
    "deadband_snap",
    "resolve_drive_profile",
    "resolve_turn_profile",
    "sample_drive",
    "sample_turn",
    "resolve_effective_speed",
]


def _selfcheck() -> None:
    """Print the three drive regimes + a turn, for a manual eyeball.

    Not a test — the real coverage is the T6 pytest suite (golden-vector parity,
    regimes, stop shape, perturbation). This is a quick ``python -m`` sanity print
    of the module against the real firmware Config, so a reader can confirm the
    kernel loads and classifies moves without wiring up pytest.
    """
    print(f"Config: {default_config()}")
    print()
    for d in (3.0, 12.0, 40.0):
        prof = resolve_drive_profile(
            d, cruise_pwm=85, start_pwm=50, end_pwm=0, profiled=True
        )
        print(
            f"{d:5.1f}cm  regime={prof.regime:<11s} "
            f"accel={prof.accel_cm:.2f} cruise={prof.cruise_cm:.2f} "
            f"decel={prof.decel_cm:.2f} peak={prof.peak_pwm:>3d} "
            f"floorX={prof.floor_crossing_cm:.2f} tail={prof.floor_tail_cm:.2f} "
            f"dur={prof.duration_s():.3f}s"
        )
    turn = resolve_turn_profile(90.0, is_pivot=False)
    pivot = resolve_turn_profile(90.0, is_pivot=True)
    print(
        f"\nturn  fast={turn.fast_pwm} slow={turn.slow_pwm} "
        f"omega={turn.omega_deg_s:.1f}deg/s dur={turn.duration_s():.3f}s"
    )
    print(
        f"pivot fast={pivot.fast_pwm} slow={pivot.slow_pwm} "
        f"omega={pivot.omega_deg_s:.1f}deg/s dur={pivot.duration_s():.3f}s"
    )


if __name__ == "__main__":
    _selfcheck()
