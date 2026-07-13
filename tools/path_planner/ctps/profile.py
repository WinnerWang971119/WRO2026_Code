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
infinite). Instead the physical velocity floors at the motor deadband. There are
TWO floors, mirroring ``Drive::_decelTailFloorMag``: the STATIC breakaway floor
``PWM_MIN_MAGNITUDE`` (35) on the accel ramp and cruise plateau, and the lower
KINETIC decel-tail floor ``PWM_MOVING_MIN`` (<= 35) once the profile enters its
decel region — ``v(s) = SCALE_CM_S_PER_PWM * max(|pwm(s)|, floor)``. Each ramp
phase gets a CLOSED-FORM duration (a log term above the floor, a linear term
inside the floor band). A profiled stop therefore renders as: decel ramp
peak -> ``PWM_MOVING_MIN``, then a constant-``PWM_MOVING_MIN`` floor-crawl to
move end, then an INSTANTANEOUS brake — a terminal keyframe that may share its
predecessor's timestamp.

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
    pwm_min: int          # PWM_MIN_MAGNITUDE (static breakaway floor, 35)
    pwm_moving_min: int   # PWM_MOVING_MIN (kinetic decel-tail floor, <= pwm_min)
    pwm_max: int          # PWM_MAX (saturation cap, 95)
    start_speed: int      # TSHAPE_START_SPEED (= PWM_MIN_MAGNITUDE)
    min_move_cm: float    # TSHAPE_MIN_MOVE_CM (5) — profiled/flat regime gate
    speed_slow: int       # SPEED_SLOW preset PWM
    speed_normal: int     # SPEED_NORMAL preset PWM (default cruise)
    speed_fast: int       # SPEED_FAST preset PWM
    line_cruise: int      # ConfigLine::CRUISE_SPEED (line-follow cruise)
    turn_cruise_pwm: int  # TURN_CRUISE_PWM
    turn_slow_pwm: int    # TURN_SLOW_SPEED_PWM
    turn_slow_zone_deg: float  # TURN_SLOW_ZONE_DEG (final-angle slow band)
    wall_push_pwm: int     # WALL_STOP_PUSH_PWM
    track_cm: float       # Config::TRACK_MM / 10 (chassis track width)
    rear_axle_offset_cm: float  # LINE_SENSOR_AXLE_OFFSET_REAR_CM (REAR overshoot window)
    hop_settle_ms: int    # HOP_SETTLE_MS (ConfigNav.h) — post-stop settle dwell


def load_profile_config(config_path: Path = DEFAULT_CONFIG_H_PATH) -> ProfileConfig:
    """Parse every Config constant the profile kernel needs, fail-loud.

    Each value is read from its OWNING firmware header via
    :mod:`tools.path_planner.config_parse` — the ONE place header parsing lives
    (issue #44). A missing/unparseable constant raises rather than falling back
    to a stale literal, surfacing at CTPS build time with the constant name +
    file (see the plan's Error Handling section).

    Constants are distributed across four headers:

    - ``ConfigDrive.h`` (``config_path``): the ``TSHAPE_*`` / ``TURN_*`` /
      ``WALL_STOP_*`` / ``SPEED_*`` / ``TRACK_MM`` knobs, the
      ``LINE_SENSOR_AXLE_OFFSET_REAR_CM`` rear overshoot window (AC8 window-fit),
      plus the symbolic ``TSHAPE_START_SPEED = PWM_MIN_MAGNITUDE``.
    - ``ConfigGlobal.h``: ``PWM_MIN_MAGNITUDE`` (static breakaway floor) /
      ``PWM_MOVING_MIN`` (kinetic decel-tail floor) / ``PWM_MAX``.
    - ``ConfigLine.h``: ``CRUISE_SPEED = SPEED_NORMAL`` (symbolic).
    - ``ConfigNav.h``: ``HOP_SETTLE_MS``, whose ``constexpr uint32_t`` RHS is the
      symbolic identifier ``HOP_SETTLE_MS_DEFAULT`` — resolved one level deep
      through that identifier's ``#define HOP_SETTLE_MS_DEFAULT <literal>``
      (not a ``constexpr``) via :mod:`tools.path_planner.config_parse`'s
      ``#define`` fallback.

    The sibling headers are resolved relative to ``config_path``'s directory so
    a test that copies a perturbed header set into a temp dir (TC2's
    ``config_path`` seam) reads a self-consistent set, while the default reads
    the real firmware headers. A perturbation test exercising ``hop_settle_ms``
    must copy ``ConfigNav.h`` into that temp dir alongside the others, or the
    ``HOP_SETTLE_MS`` read raises (no fallback literal, issue #44). The
    symbolic-RHS search set is likewise rooted at ``config_path``'s directory so
    ``TSHAPE_START_SPEED = PWM_MIN_MAGNITUDE``, ``CRUISE_SPEED = SPEED_NORMAL``,
    and ``HOP_SETTLE_MS = HOP_SETTLE_MS_DEFAULT`` all resolve against the same
    (possibly temp) headers rather than the real ones.

    :param config_path: the ``ConfigDrive.h`` holding the drive constants;
        defaults to the real header resolved relative to this module
        (cwd-independent). Its parent directory must also hold ``ConfigGlobal.h``,
        ``ConfigLine.h``, and ``ConfigNav.h``.
    :return: a fully-populated :class:`ProfileConfig`.
    :raises OSError: when ``config_path`` (or a sibling / symbolic-RHS search
        header) is unreadable.
    :raises ValueError: when a required constant is missing or unparseable.
    """
    from tools.path_planner.planner import parse_track_cm_from_config

    config_dir = config_path.parent
    global_h = config_dir / "ConfigGlobal.h"
    line_h = config_dir / "ConfigLine.h"
    nav_h = config_dir / "ConfigNav.h"
    # Symbolic RHS (TSHAPE_START_SPEED = PWM_MIN_MAGNITUDE, CRUISE_SPEED =
    # SPEED_NORMAL, HOP_SETTLE_MS = HOP_SETTLE_MS_DEFAULT) resolves one level
    # across this set, rooted at config_dir.
    search = (config_path, global_h, line_h, nav_h)

    return ProfileConfig(
        accel=parse_config_float("TSHAPE_ACCEL_PWM_PER_CM", config_path),
        ramp_min=parse_config_float("TSHAPE_RAMP_MIN_CM", config_path),
        ramp_max=parse_config_float("TSHAPE_RAMP_MAX_CM", config_path),
        no_ramp_eps=parse_config_int("TSHAPE_NO_RAMP_EPSILON_PWM", config_path, search),
        pwm_min=parse_config_int("PWM_MIN_MAGNITUDE", global_h, search),
        pwm_moving_min=parse_config_int("PWM_MOVING_MIN", global_h, search),
        pwm_max=parse_config_int("PWM_MAX", global_h, search),
        start_speed=parse_config_int("TSHAPE_START_SPEED", config_path, search),
        min_move_cm=parse_config_float("TSHAPE_MIN_MOVE_CM", config_path),
        speed_slow=parse_config_int("SPEED_SLOW", config_path, search),
        speed_normal=parse_config_int("SPEED_NORMAL", config_path, search),
        speed_fast=parse_config_int("SPEED_FAST", config_path, search),
        line_cruise=parse_config_int("CRUISE_SPEED", line_h, search),
        turn_cruise_pwm=parse_config_int("TURN_CRUISE_PWM", config_path, search),
        turn_slow_pwm=parse_config_int("TURN_SLOW_SPEED_PWM", config_path, search),
        turn_slow_zone_deg=parse_config_float("TURN_SLOW_ZONE_DEG", config_path),
        wall_push_pwm=parse_config_int("WALL_STOP_PUSH_PWM", config_path, search),
        track_cm=parse_track_cm_from_config(config_path),
        rear_axle_offset_cm=parse_config_float(
            "LINE_SENSOR_AXLE_OFFSET_REAR_CM", config_path
        ),
        hop_settle_ms=parse_config_int("HOP_SETTLE_MS", nav_h, search),
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
#   v(s) = SCALE_CM_S_PER_PWM * max(|pwm(s)|, floor)
# where pwm(s) is LINEAR in distance along a ramp and `floor` is the STATIC
# breakaway floor (PWM_MIN_MAGNITUDE) on the accel ramp / cruise plateau, or the
# lower KINETIC decel-tail floor (PWM_MOVING_MIN) on the decel ramp / floor-crawl
# tail (mirrors Drive::_decelTailFloorMag). The duration of a ramp is the
# closed-form integral of ds/v(s), split at the point where |pwm| crosses that
# floor. This is what makes an endSpeed=0 stop FINITE (the naive dt = ds/(k*pwm)
# diverges near a linear zero).


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
    :param floor: deadband floor magnitude to integrate against; the caller
        passes the STATIC breakaway floor (``PWM_MIN_MAGNITUDE``) for an accel
        ramp or the KINETIC decel-tail floor (``PWM_MOVING_MIN``) for a decel
        ramp; > 0.
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

        [0, accel_cm)                 ramp up   start_pwm -> peak_pwm (static floor)
        [accel_cm, accel_cm+cruise_cm) cruise   hold at peak_pwm (static floor)
        [.., floor_crossing_cm)       ramp down peak_pwm -> kinetic floor
        [floor_crossing_cm, dist_cm)  floor-crawl at PWM_MOVING_MIN (kinetic floor)
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

        ``v(s) = SCALE_CM_S_PER_PWM * max(|pwm_at_s(s)|, floor)``, where ``floor``
        is the KINETIC floor (``PWM_MOVING_MIN``) once ``s`` is in the decel
        region, and the STATIC breakaway floor (``PWM_MIN_MAGNITUDE``) on the
        accel ramp and the cruise plateau — mirroring the firmware's
        ``Drive::_decelTailFloorMag`` (kinetic floor applies ONLY in the profile
        decel region). The floor is what keeps a stop finite: at the end of a
        profiled move the PWM ramps to 0 but the chassis still crawls at the
        decel-tail floor until the terminal brake.

        Uses the int-truncated kernel PWM (:meth:`pwm_at_s`) so a displayed
        velocity readout matches the firmware setpoint at that point. Phase
        DURATIONS instead integrate the exact-float PWM
        (:meth:`_pwm_mag_exact_at_s`) so a keyframe's timestamp lands smoothly on
        the profile curve; the two agree to sub-PWM and are both display-only.

        :param s: distance along the move (cm).
        :return: strictly-positive display velocity in cm/s.
        """
        mag = abs(self.pwm_at_s(s))
        decel_start = self.dist_cm - self.decel_cm
        in_decel = self.decel_cm > 0.0 and s >= decel_start
        floor = self._cfg.pwm_moving_min if in_decel else self._cfg.pwm_min
        floored = max(mag, floor)
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
        # Static breakaway floor — accel ramp and cruise plateau. The decel side
        # (ramp-down + floor-crawl tail) instead floors at the KINETIC decel-tail
        # floor (PWM_MOVING_MIN), mirroring Drive::_decelTailFloorMag: the kinetic
        # floor applies ONLY in the profile decel region.
        floor = float(cfg.pwm_min)
        decel_floor = float(cfg.pwm_moving_min)

        if self.dist_cm <= 0.0:
            return []

        if not self.profiled:
            mag = max(abs(self.cruise_pwm), cfg.pwm_min)
            return [(0.0, self.dist_cm, self.dist_cm / (scale * mag))]

        segments: List[Tuple[float, float, float]] = []
        # Ramp-up: start_pwm -> peak_pwm over [0, accel_cm). Static floor.
        a_end = self.accel_cm
        if a_end > 0.0:
            dur = _linear_ramp_duration(
                a_end, abs(self.start_pwm), abs(self.peak_pwm), scale, floor
            )
            segments.append((0.0, a_end, dur))
        # Cruise: constant peak over [accel_cm, accel_cm + cruise_cm). Static floor.
        c_end = a_end + self.cruise_cm
        if self.cruise_cm > 0.0:
            mag = max(abs(self.peak_pwm), cfg.pwm_min)
            segments.append((a_end, c_end, self.cruise_cm / (scale * mag)))
        # Ramp-down above the floor: peak -> max(|end_pwm|, decel_floor) over
        # [c_end, floor_crossing_cm). Timing the descent to the TRUE end magnitude
        # (not always the deadband floor) keeps duration_s() consistent with the
        # interior keyframes, whose _pwm_mag_exact_at_s ramps to |end_pwm|. When the
        # move ends at/above the floor (e.g. setEndSpeed(60)) floor_crossing_cm ==
        # dist_cm (no crawl tail), so this spans the whole decel and lands on
        # |end_pwm|; when it ends below the floor the ramp still bottoms at the
        # KINETIC floor and the floor-crawl tail below carries the rest. The end=0
        # stop is unchanged in shape, only the floor value moves.
        above_len = self.floor_crossing_cm - c_end
        if above_len > 0.0:
            decel_target = max(abs(self.end_pwm), decel_floor)
            dur = _linear_ramp_duration(
                above_len, abs(self.peak_pwm), decel_target, scale, decel_floor
            )
            segments.append((c_end, self.floor_crossing_cm, dur))
        # Floor-crawl tail: constant KINETIC floor over [floor_crossing_cm, dist_cm).
        if self.floor_tail_cm > 0.0:
            segments.append(
                (
                    self.floor_crossing_cm,
                    self.dist_cm,
                    self.floor_tail_cm / (scale * decel_floor),
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
    rear_window_cm: Optional[float] = None,
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
    the pre-snap PWM magnitude first drops to the KINETIC decel-tail floor
    (``pwm_moving_min``), mirroring ``Drive::_decelTailFloorMag``. From that
    point to ``dist_cm`` the chassis crawls at that kinetic floor, then brakes.

    LineFollower window-fit (AC8 / TC9)
    -----------------------------------
    A braked stop sheds ``peak -> PWM_MOVING_MIN`` (the kinetic decel-tail floor)
    then brakes; the natural shed distance is ``(|peak| - floor) / accel`` (8 cm
    at FAST cruise 90, floor = the kinetic floor).
    A REAR-led final hop only has ``LINE_SENSOR_AXLE_OFFSET_REAR_CM`` (~7.68 cm) of
    overshoot window, so the firmware FITS the ramp-down to the window
    (``LineFollower.cpp:287-301``: ``rampDownCm = min(defaultRampCm, window)``) — a
    STEEPER ramp that still reaches the floor before the brake, rather than an
    unfitted 8 cm shed that would overrun. When ``rear_window_cm`` is given AND the
    natural above-floor shed exceeds it, this reshapes the display decel to that
    window-fitted shed: the ``peak -> floor`` descent occupies exactly
    ``rear_window_cm`` ending AT the gate (``floor_crossing_cm == dist_cm``, no
    floor-crawl tail), so the render shows the steeper-but-reaches-floor shed
    (AC8). This is a RENDER-only reshape of the display phase boundaries — the
    golden-pinned ``resolve_tshape_ramps`` kernel is untouched — and it fires only
    for a braked stop (``end_pwm == 0``); it is NOT a build-failing assert (the
    firmware degrades gracefully, so CTPS mirrors that degradation).

    :param dist_cm: total travel distance (cm).
    :param cruise_pwm: signed cruise PWM (sign = direction).
    :param start_pwm: signed PWM at move start.
    :param end_pwm: signed PWM at move end.
    :param profiled: ``True`` for a ``.tShape()`` move, ``False`` for ``.flat()``.
    :param rear_window_cm: when set (a REAR-led braked final hop), the available
        rear overshoot window (cm) the ``peak -> floor`` shed is FITTED to; the
        shed is compressed to a steeper ramp reaching the floor at the gate when
        the natural shed would exceed it. ``None`` (the default, a FRONT-led or
        non-stop move) leaves the natural shed + floor-crawl tail.
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
        dist, accel_cm, decel_cm, peak_pwm, start_pwm, end_pwm, cfg.pwm_moving_min
    )
    floor_tail_cm = dist - floor_crossing_cm
    if floor_tail_cm < 0.0:
        floor_tail_cm = 0.0

    eff_end_pwm = int(end_pwm)

    # LineFollower window-fit (AC8 / TC9). A REAR-led braked final hop only has
    # ``rear_window_cm`` of overshoot room; when the natural ``peak -> floor`` shed
    # would exceed it, the firmware fits the ramp-down to the window (steeper ramp,
    # reaches floor AT the gate, no crawl tail). Reshape the DISPLAY decel to that
    # window-fitted shed. Render-only: the golden kernel above is untouched.
    if rear_window_cm is not None and int(end_pwm) == 0:
        (
            decel_cm,
            cruise_cm,
            floor_crossing_cm,
            floor_tail_cm,
            eff_end_pwm,
        ) = _window_fit_stop(
            dist,
            accel_cm,
            decel_cm,
            cruise_cm,
            peak_pwm,
            floor_crossing_cm,
            floor_tail_cm,
            float(rear_window_cm),
            cfg.pwm_moving_min,
        )

    return DriveProfile(
        dist_cm=dist,
        cruise_pwm=int(cruise_pwm),
        start_pwm=int(start_pwm),
        end_pwm=eff_end_pwm,
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


def _window_fit_stop(
    dist_cm: float,
    accel_cm: float,
    decel_cm: float,
    cruise_cm: float,
    peak_pwm: int,
    floor_crossing_cm: float,
    floor_tail_cm: float,
    rear_window_cm: float,
    floor_mag: int,
) -> Tuple[float, float, float, float, int]:
    """Reshape a braked stop's display decel to the LineFollower window-fit (AC8).

    Mirrors ``LineFollower.cpp:287-301`` in the DISPLAY layer: a REAR-led braked
    final hop sheds ``peak -> floor`` over the LAST ``rear_window_cm`` before the
    distance gate, reaching the floor AT the gate then braking — no floor-crawl
    tail. When the natural above-floor shed (the ``peak -> floor`` portion of the
    resolved decel ramp) already fits inside ``rear_window_cm``, the passed-in
    natural values are returned UNCHANGED (the firmware's ``min(defaultRampCm,
    window)`` picks the smaller natural shed, and ``end_pwm`` stays the 0-brake).
    When the natural shed OVERRUNS the window, the shed is compressed to
    ``rear_window_cm`` (a steeper ramp), the floor-crossing is pinned to the gate,
    the crawl tail is dropped, and the display ``end_pwm`` is set to the
    sign-corrected floor so the decel display segment ramps ``peak -> floor`` (not
    to 0) and lands at floor speed at the gate, matching the firmware ramp-down.
    ``floor_mag`` is the KINETIC decel-tail floor (``PWM_MOVING_MIN``) the caller
    passes for a braked stop — the window-fit ends at that kinetic floor (TC10).

    :param dist_cm: total move distance (cm).
    :param accel_cm: the resolved ramp-up distance (unchanged by the fit).
    :param decel_cm: the resolved natural decel-ramp distance (``peak -> 0``).
    :param cruise_cm: the resolved natural cruise-plateau distance.
    :param peak_pwm: signed peak PWM (its sign carries the run direction).
    :param floor_crossing_cm: the resolved natural floor-crossing distance.
    :param floor_tail_cm: the resolved natural floor-crawl tail length.
    :param rear_window_cm: the available rear overshoot window (cm); non-positive
        or larger-than-natural leaves the shed unchanged.
    :param floor_mag: the floor magnitude to fit/solve against (the caller passes
        the kinetic decel-tail floor for a braked stop).
    :return: ``(decel_cm, cruise_cm, floor_crossing_cm, floor_tail_cm,
        eff_end_pwm)`` — the reshaped display fields; identical to the passed-in
        natural values (with ``eff_end_pwm == 0``) when no fit is needed.
    """
    peak_mag = abs(int(peak_pwm))
    floor = float(floor_mag)

    # The natural above-floor shed (peak -> floor) of the resolved decel ramp: the
    # fraction of the peak->0 ramp spent above the floor. A peak already at/below
    # the floor has no shed to fit.
    if decel_cm <= 0.0 or peak_mag <= floor:
        natural_shed = 0.0
    else:
        natural_shed = decel_cm * (peak_mag - floor) / peak_mag

    # Not binding: window >= natural shed, or degenerate. Return the natural values
    # unchanged (a braked stop keeps end_pwm == 0 + its floor-crawl tail).
    if rear_window_cm <= 0.0 or natural_shed <= rear_window_cm:
        return decel_cm, cruise_cm, floor_crossing_cm, floor_tail_cm, 0

    # Overrun: compress the shed to the window. The steeper ramp reaches the floor
    # AT the gate; there is no crawl tail. The decel now spans exactly the window.
    fitted_decel = rear_window_cm
    fitted_cruise_cm = dist_cm - accel_cm - fitted_decel
    if fitted_cruise_cm < 0.0:
        fitted_cruise_cm = 0.0
    # End at the floor (sign-corrected) so the display decel ramps peak -> floor
    # and lands at floor speed at the gate, then brakes.
    eff_end_pwm = -int(floor_mag) if peak_pwm < 0 else int(floor_mag)
    return fitted_decel, fitted_cruise_cm, dist_cm, 0.0, eff_end_pwm


def _decel_floor_crossing(
    dist_cm: float,
    accel_cm: float,
    decel_cm: float,
    peak_pwm: int,
    start_pwm: int,
    end_pwm: int,
    floor_mag: int,
) -> float:
    """Distance at which the decel ramp's |PWM| first drops to the deadband floor.

    From this distance to ``dist_cm`` the chassis crawls at ``floor_mag`` (the
    floor-crawl tail) before the terminal brake. If the move never rises above /
    never drops to the floor on the decel side (e.g. ``end_pwm`` magnitude is
    already >= floor, so there is no sub-floor tail), the crossing is ``dist_cm``
    (no tail). The caller passes the KINETIC decel-tail floor (``PWM_MOVING_MIN``)
    for this crossing, mirroring ``Drive::_decelTailFloorMag``.

    The decel ramp is linear in distance from ``peak_pwm`` at
    ``dist_cm - decel_cm`` to ``end_pwm`` at ``dist_cm``. Solve for where its
    magnitude equals ``floor_mag``. Uses the display (float64) linear PWM, not
    the int-truncated :func:`tshape_speed_at`, so the crossing is smooth; the
    tiny difference from the int-cast curve is display-only and sub-pixel.

    :param dist_cm: total move distance (cm).
    :param accel_cm: resolved ramp-up distance.
    :param decel_cm: resolved ramp-down distance.
    :param peak_pwm: signed peak PWM.
    :param start_pwm: signed start PWM.
    :param end_pwm: signed end PWM.
    :param floor_mag: the floor magnitude to solve against (the caller passes the
        kinetic decel-tail floor).
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
    if end_mag >= floor_mag:
        # Never drops below the floor — no sub-floor crawl tail.
        return dist_cm
    if peak_mag <= floor_mag:
        # Whole decel ramp is already at/below the floor: the crawl tail begins
        # at the start of the decel ramp.
        return dist_cm - decel_cm

    # Linear descent from peak_mag to end_mag; find the fraction into the ramp
    # where the magnitude hits floor_mag.
    frac = (peak_mag - floor_mag) / (peak_mag - end_mag)
    return (dist_cm - decel_cm) + frac * decel_cm


# ==============================================================================
# TurnProfile — Config-wired turn / pivot (snapped per-stage PWM + rate)
# ==============================================================================


@dataclass(frozen=True)
class TurnProfile:
    """A resolved turn or pivot: a config-derived TWO-STAGE (cruise -> slow) profile.

    Produced by :func:`resolve_turn_profile`. Mirrors the firmware TurnGyro FSM
    (``Drive.cpp::_updateTurnProfile``), which runs the swing in two stages:

    - **CRUISE** at ``TURN_CRUISE_PWM`` while the REMAINING angle to target is
      ``> TURN_SLOW_ZONE_DEG``.
    - **SLOW** at ``TURN_SLOW_SPEED_PWM`` (not overridable) once the remaining
      angle is ``<= TURN_SLOW_ZONE_DEG``, i.e. the FINAL ``TURN_SLOW_ZONE_DEG``
      degrees of the swing.

    Both stage PWMs go through ``deadbandSnap`` on emit (``Drive.cpp:1173-1174``),
    so ``fast_pwm`` / ``slow_pwm`` are the SNAPPED per-stage magnitudes the
    firmware actually commands. ``theta_deg`` is the signed swing (CCW positive).

    The swept angle splits into two segments (both non-negative, summing to
    ``|theta|``):

    - ``slow_deg`` = ``min(|theta|, TURN_SLOW_ZONE_DEG)`` — the final settle band.
    - ``cruise_deg`` = ``max(0, |theta| - TURN_SLOW_ZONE_DEG)`` — everything before.

    Each stage has its own derived angular rate (``omega_cruise`` / ``omega_slow``):
    a wheel at PWM ``p`` moves ``v = SCALE * p`` cm/s, a counter-rotation spins
    both wheels so ``omega = (2*v / TRACK_CM) * 180/pi``; a pivot drives ONE
    wheel about the other, HALVING the rate.

    With the DEFAULT Config (``TURN_CRUISE_PWM = 50``, ``TURN_SLOW_SPEED_PWM =
    20``, ``PWM_MIN_MAGNITUDE = 50``) both stages snap to ``|PWM| = 50``, so
    ``omega_cruise == omega_slow`` and the cumulative-time-over-angle function
    collapses to the single uniform rate ``phi / omega`` — a CONSTANT-RATE arc,
    byte-identical to the pre-two-stage constant-rate render. A genuine two-stage
    ramp appears only when the Config makes the stages snap to different PWMs
    (e.g. ``TURN_CRUISE_PWM`` raised above the ``50`` floor while the slow stage
    stays at it) — then the final ``TURN_SLOW_ZONE_DEG`` degrees take
    proportionally longer per degree than the cruise segment. Per-turn *builder*
    overrides (``turnDeg.setPower``) remain deferred (the recorder never captures
    them); the two-stage uses the Config default cruise, which CTPS parses.
    """

    theta_deg: float
    is_pivot: bool
    fast_pwm: int
    slow_pwm: int
    cruise_deg: float
    slow_deg: float
    omega_cruise_deg_s: float
    omega_slow_deg_s: float
    _cfg: ProfileConfig = dc_field(repr=False)

    def duration_s(self) -> float:
        """Total display duration of the turn (seconds), from the two-stage integral.

        ``cruise_deg / omega_cruise + slow_deg / omega_slow``. Under the default
        Config both omegas are equal, so this reduces to ``|theta| / omega`` — the
        old constant-rate duration, unchanged. A zero-angle turn (or a degenerate
        zero rate on a used stage) contributes 0 s to that stage (no
        divide-by-zero — a stage with zero swept angle is skipped, and the omega
        guard is in :func:`resolve_turn_profile`).

        :return: non-negative turn duration in seconds.
        """
        return self.time_at_angle(abs(self.theta_deg))

    def time_at_angle(self, phi: float) -> float:
        """Cumulative time (seconds) to sweep ``phi`` degrees from the turn start.

        Two-stage: the CRUISE segment (the first ``cruise_deg`` degrees) runs at
        ``omega_cruise``, then the SLOW segment (the final ``slow_deg`` degrees)
        runs at ``omega_slow``. For ``phi`` in ``[0, |theta|]``:

        - ``phi <= cruise_deg`` -> ``phi / omega_cruise``
        - else -> ``cruise_deg / omega_cruise + (phi - cruise_deg) / omega_slow``

        ``phi`` is clamped into ``[0, |theta|]``. Under the default Config
        (equal omegas) this is exactly ``phi / omega`` for every ``phi`` — so a
        keyframe re-timed by its cumulative angle lands on the SAME timestamp the
        old uniform grid produced (the byte-identity invariant).

        :param phi: swept angle so far (deg, non-negative, magnitude).
        :return: non-negative cumulative time in seconds.
        """
        abs_theta = abs(self.theta_deg)
        if abs_theta <= 0.0:
            return 0.0
        p = 0.0 if phi < 0.0 else (abs_theta if phi > abs_theta else phi)

        if p <= self.cruise_deg:
            if self.omega_cruise_deg_s <= 0.0:
                return 0.0
            return p / self.omega_cruise_deg_s

        t = 0.0
        if self.cruise_deg > 0.0 and self.omega_cruise_deg_s > 0.0:
            t += self.cruise_deg / self.omega_cruise_deg_s
        if self.omega_slow_deg_s > 0.0:
            t += (p - self.cruise_deg) / self.omega_slow_deg_s
        return t


def resolve_turn_profile(
    theta_deg: float,
    *,
    is_pivot: bool,
    cruise_pwm: Optional[int] = None,
    slow_pwm: Optional[int] = None,
    cfg: Optional[ProfileConfig] = None,
) -> TurnProfile:
    """Resolve a turn / pivot into a config-derived two-stage :class:`TurnProfile`.

    Per the plan Decisions (AC3), each stage's PWM is the deadband-SNAPPED command
    the firmware emits, NOT the raw ``TURN_*`` Config value. The swept angle is
    split into a CRUISE segment (all but the final ``TURN_SLOW_ZONE_DEG`` degrees)
    at ``TURN_CRUISE_PWM`` and a SLOW segment (the final ``TURN_SLOW_ZONE_DEG``)
    at ``TURN_SLOW_SPEED_PWM``, mirroring ``Drive.cpp::_updateTurnProfile``.

    With the default Config (``TURN_CRUISE_PWM = 50``, ``TURN_SLOW_SPEED_PWM =
    20``, ``PWM_MIN_MAGNITUDE = 50``) both stages snap to ``|PWM| = 50`` -> the
    two stage omegas are EQUAL, so the two-stage integral reduces to the single
    uniform rate and the render is byte-identical to the old constant-rate arc. A
    genuine two-stage ramp appears only when the Config makes the snapped stage
    PWMs differ (e.g. ``TURN_CRUISE_PWM`` raised above the floor).

    Each stage's angular rate is DERIVED from the single visual scale + the
    chassis track (no second scale constant): a wheel at PWM ``p`` moves at
    ``v = SCALE * p`` cm/s; a counter-rotation spins both wheels, so
    ``omega = (2 * v / TRACK_CM) * 180/pi``; a pivot drives ONE wheel about the
    other, HALVING the rate — consistent for BOTH stages.

    :param theta_deg: signed swing angle (deg, CCW positive).
    :param is_pivot: ``True`` for a single-wheel pivot (rate halved).
    :param cruise_pwm: fast-stage PWM override; ``None`` uses ``TURN_CRUISE_PWM``.
        (Not wired to any recorded builder override — turn-override capture is
        deferred; this arg exists for tests / a future follow-up.)
    :param slow_pwm: slow-stage PWM override; ``None`` uses ``TURN_SLOW_SPEED_PWM``.
    :param cfg: parsed Config; defaults to :func:`default_config`.
    :return: the resolved :class:`TurnProfile`.
    """
    cfg = cfg or default_config()

    raw_fast = cfg.turn_cruise_pwm if cruise_pwm is None else int(cruise_pwm)
    raw_slow = cfg.turn_slow_pwm if slow_pwm is None else int(slow_pwm)

    fast_pwm = deadband_snap(raw_fast, cfg.pwm_min, cfg.pwm_max)
    slow_pwm = deadband_snap(raw_slow, cfg.pwm_min, cfg.pwm_max)

    # Split the swept angle into the final slow-zone band and the cruise remainder.
    abs_theta = abs(float(theta_deg))
    slow_zone = cfg.turn_slow_zone_deg
    if slow_zone < 0.0:
        slow_zone = 0.0
    slow_deg = min(abs_theta, slow_zone)
    cruise_deg = abs_theta - slow_deg

    omega_cruise = _turn_omega(fast_pwm, cfg.track_cm, is_pivot)
    omega_slow = _turn_omega(slow_pwm, cfg.track_cm, is_pivot)

    return TurnProfile(
        theta_deg=float(theta_deg),
        is_pivot=is_pivot,
        fast_pwm=fast_pwm,
        slow_pwm=slow_pwm,
        cruise_deg=cruise_deg,
        slow_deg=slow_deg,
        omega_cruise_deg_s=omega_cruise,
        omega_slow_deg_s=omega_slow,
        _cfg=cfg,
    )


def _turn_omega(snapped_pwm: int, track_cm: float, is_pivot: bool) -> float:
    """Derived angular rate (deg/s) for a turn stage at a snapped PWM magnitude.

    A wheel commanded at ``|pwm|`` moves ``v = SCALE_CM_S_PER_PWM * |pwm|`` cm/s.
    A counter-rotating turn spins both wheels, so the chassis angular rate is
    ``omega = (2 * v / track_cm)`` rad/s, converted to deg/s. A pivot drives ONE
    wheel about the planted other, HALVING the rate. A zero PWM (a stage the
    firmware would not command) yields 0 — the caller's cumulative-time function
    skips a zero-rate stage rather than dividing by it.

    :param snapped_pwm: the deadband-snapped stage PWM (signed; magnitude used).
    :param track_cm: chassis track width (cm).
    :param is_pivot: ``True`` to halve the rate for a single-wheel pivot.
    :return: non-negative angular rate in deg/s.
    """
    if track_cm <= 0.0:
        raise ValueError(
            f"_turn_omega: track_cm must be positive, got {track_cm!r} — a zero or "
            "negative chassis track width (Config::TRACK_MM) makes the turn rate "
            "undefined. Check the ConfigDrive.h fed to resolve_turn_profile."
        )
    wheel_cm_s = SCALE_CM_S_PER_PWM * abs(snapped_pwm)
    omega = degrees(2.0 * wheel_cm_s / track_cm)
    if is_pivot:
        omega *= 0.5
    return omega


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


def sample_drive_run(
    profile: DriveProfile,
    segments: List[Tuple[Tuple[float, float, float], Tuple[float, float, float], float]],
    t0: float,
) -> List[List[Dict[str, Any]]]:
    """Slice ONE run :class:`DriveProfile` into per-segment sub-keyframe lists.

    The CTPS run-grouper (``track_builder``) merges a whole collinear
    pass-through run (a chain of geometry primitives bounded by braked stops)
    into a SINGLE velocity profile over the run's summed chord distance, so the
    render is one ramp-up / cruise / ramp-down trapezoid instead of a per-node
    dip-and-stop. This function is the slicer: it evaluates that single
    ``profile`` at each segment's cumulative-distance window and returns, per
    segment, the dense sub-keyframes whose timestamps follow the ONE run profile
    (never re-ramping mid-run) and whose LAST sub-keyframe pins that segment's
    end pose byte-exact (the TC7 pose-pin invariant — only the velocity curve
    BETWEEN pose boundaries changes, never where the chassis is at each node).

    Each ``segments`` entry is ``(start_pose, end_pose, seg_dist_cm)``:

    * ``start_pose`` / ``end_pose`` — the ``(x, y, heading_deg)`` at the segment
      boundaries (the previous keyframe's pose and this primitive's sim-snapshot
      pose). Poses are interpolated along THIS segment's own chord
      (``start_pose -> end_pose``), NOT the whole run's chord, so a run whose
      segments are only near-collinear still pins every snapshot pose exactly.
    * ``seg_dist_cm`` — the segment's own chord length; the cumulative sum over
      the segments equals ``profile.dist_cm`` (the caller builds ``profile`` over
      exactly that sum).

    The returned list is parallel to ``segments`` (one sub-keyframe list each).
    As with :func:`sample_drive`, the FIRST sub-keyframe of the FIRST segment
    duplicates the caller's previous keyframe (same ``t0`` + pose) and is DROPPED
    here, so each returned list is the REPLACEMENT run for its geometry step — an
    empty list for a zero-length segment. Timestamps are non-decreasing and
    profile-derived (dense near the single run's ramps).

    :param profile: the resolved :class:`DriveProfile` for the WHOLE run (its
        ``dist_cm`` equals the sum of the ``seg_dist_cm`` values).
    :param segments: the ordered per-segment ``(start_pose, end_pose, seg_dist_cm)``
        boundaries, summing to ``profile.dist_cm``.
    :param t0: timestamp of the run-start keyframe (the running clock before the
        first segment).
    :return: a list parallel to ``segments``; entry ``k`` is the replacement
        sub-keyframes for segment ``k`` (first-of-run duplicate dropped).
    """
    if not segments:
        return []

    total = profile.dist_cm
    time_at = _cumulative_time_fn(profile)

    out: List[List[Dict[str, Any]]] = []
    s_lo = 0.0
    for (start_pose, end_pose, seg_dist) in segments:
        seg = max(0.0, float(seg_dist))
        s_hi = s_lo + seg
        # Clamp the top of the LAST segment to the profile's own distance so
        # float accumulation cannot push the final window past dist_cm (which
        # would clip its terminal brake keyframe).
        if s_hi > total:
            s_hi = total

        sub = _sample_run_segment(
            profile, start_pose, end_pose, s_lo, s_hi, t0, time_at
        )
        # Drop each segment's LEADING boundary keyframe: for the first segment it
        # duplicates the caller's previous keyframe (same t0 + pose); for every
        # subsequent segment it duplicates the PRIOR segment's last sub-keyframe
        # (the shared node boundary is sampled once per adjacent window). Keeping
        # it would leave a zero-length duplicate keyframe at each interior node —
        # cosmetically a "stop"-looking pair even though velocity holds cruise.
        # The segment's END pose (``sub[-1]``) is preserved, so every node pose
        # still byte-pins (TC7).
        out.append(sub[1:])
        s_lo = s_hi

    return out


def _sample_run_segment(
    profile: DriveProfile,
    start_pose: Tuple[float, float, float],
    end_pose: Tuple[float, float, float],
    s_lo: float,
    s_hi: float,
    t0: float,
    time_at,
) -> List[Dict[str, Any]]:
    """Sample ONE run segment's window ``[s_lo, s_hi]`` of a shared profile.

    Mirrors :func:`sample_drive`'s densification, but over a sub-window of the
    run profile: sample distances are the profile phase boundaries plus interior
    ramp samples that fall inside ``[s_lo, s_hi]`` (so a segment that straddles a
    ramp still tracks the curve), each mapped to its cumulative run time via
    ``time_at`` and to a pose LERPed along this segment's own chord by the
    fraction into the segment. The first sample is pinned to ``start_pose`` and
    the last to ``end_pose`` byte-exact.

    :param profile: the shared run :class:`DriveProfile`.
    :param start_pose: ``(x, y, heading_deg)`` at ``s_lo`` (segment start).
    :param end_pose: ``(x, y, heading_deg)`` at ``s_hi`` (segment end / snapshot).
    :param s_lo: window start distance along the run (cm).
    :param s_hi: window end distance along the run (cm).
    :param t0: run-start clock (the profile times are relative to this).
    :param time_at: the run profile's ``s -> cumulative time`` closure.
    :return: an ordered keyframe list whose first pose is ``start_pose`` and last
        pose is ``end_pose``; a zero-length window yields a single keyframe.
    """
    x0, y0, h0 = start_pose
    x1, y1, h1 = end_pose
    dh = _shortest_arc(h0, h1)

    seg_len = s_hi - s_lo
    if seg_len <= 0.0:
        # Zero-length window: a single keyframe at the (equal) boundary pose.
        return [_pose_kf(t0 + time_at(s_lo), x0, y0, h0)]

    # Ordered run sample distances that fall inside this window, plus the window
    # endpoints. Reuse the whole-run boundary/ramp sampler then filter — this
    # keeps the interior spacing (RAMP_SAMPLE_STEP_CM / ceiling) identical to the
    # per-primitive path a single-segment run would take.
    run_distances = _drive_sample_distances(profile)
    distances = {s_lo, s_hi}
    for d in run_distances:
        if s_lo < d < s_hi:
            distances.add(d)
    ordered = sorted(distances)

    keyframes: List[Dict[str, Any]] = []
    for s in ordered:
        frac = (s - s_lo) / seg_len
        x = x0 + (x1 - x0) * frac
        y = y0 + (y1 - y0) * frac
        h = (h0 + dh * frac) % 360.0
        keyframes.append(_pose_kf(t0 + time_at(s), x, y, h))

    # Pin the segment endpoints byte-exact so the run's per-node poses match the
    # sim snapshots (TC7) even under float lerp drift at the window edges.
    keyframes[0] = _pose_kf(keyframes[0]["t"], x0, y0, h0)
    keyframes[-1] = _pose_kf(keyframes[-1]["t"], x1, y1, h1)
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

    Each phase's partial duration integrates against ITS OWN floor — the static
    breakaway floor (``pwm_min``) for the accel ramp / cruise plateau, the
    kinetic decel-tail floor (``pwm_moving_min``) for the decel ramp / floor-crawl
    tail — so an interior keyframe landing inside the decel phase is timed
    consistently with that phase's own closed-form duration (:meth:`DriveProfile.
    _phase_segments`), preserving the "partials sum to the full duration"
    invariant.

    :param profile: the resolved drive profile.
    :return: a callable ``time_at(s) -> float`` non-decreasing over ``[0, dist]``.
    """
    segments = profile._phase_segments()
    cfg = profile._cfg
    scale = SCALE_CM_S_PER_PWM
    static_floor = float(cfg.pwm_min)
    decel_floor = float(cfg.pwm_moving_min)
    decel_start = profile.dist_cm - profile.decel_cm

    # Precompute the cumulative time at the START of each segment, along with the
    # floor that segment integrates against (kinetic on the decel side, static
    # elsewhere).
    starts: List[Tuple[float, float, float, float, float]] = []  # (s0, s1, t0_seg, dur, floor)
    t_acc = 0.0
    for (s0, s1, dur) in segments:
        seg_floor = decel_floor if (profile.decel_cm > 0.0 and s0 >= decel_start) else static_floor
        starts.append((s0, s1, t_acc, dur, seg_floor))
        t_acc += dur
    total_t = t_acc

    def time_at(s: float) -> float:
        if s <= 0.0:
            return 0.0
        if s >= profile.dist_cm:
            return total_t
        for (s0, s1, t0_seg, dur, seg_floor) in starts:
            if s <= s1 or (s1 == s0):
                if s <= s0:
                    return t0_seg
                # Partial closed-form duration from s0 to s within this phase.
                return t0_seg + _partial_phase_duration(
                    profile, s0, s1, s, scale, seg_floor
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
    :param floor: deadband floor magnitude for THIS phase — the caller passes
        the kinetic decel-tail floor on the decel side, the static breakaway
        floor elsewhere (see :func:`_cumulative_time_fn`).
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
    signed_theta_deg: Optional[float] = None,
) -> List[Dict[str, Any]]:
    """Sample a turn / pivot into keyframes timed by the two-stage turn profile.

    Two modes:

    1. ``arc_keyframes`` given (the pivot-arc path): T10 has already computed the
       geometric arc sub-keyframes (via ``_build_pivot_arc``), evenly spaced in
       swept angle. This RE-TIMES them — poses are left byte-for-byte untouched —
       by each keyframe's CUMULATIVE swept angle through the two-stage
       cruise->slow function (:meth:`TurnProfile.time_at_angle`): sub-keyframe
       ``k`` of ``N`` is at swept angle ``(k/N) * |theta|`` and gets timestamp
       ``t0 + time_at_angle((k/N)*|theta|)``. The final sub-keyframe's pose is
       preserved byte-for-byte.
    2. ``arc_keyframes`` None (an in-place turn): emits keyframes interpolating
       ``start_pose -> end_pose`` (position lerp — usually a pure heading change
       so x/y are equal — heading on the signed swing), each timed by its
       cumulative swept angle through the same two-stage function. Sample count
       tracks ``ARC``-style density (denser than one keyframe per stage so the
       cruise->slow rate change is visible), capped at ``KEYFRAME_COUNT_CEILING``.

    Under the default Config both stages snap to ``|PWM| = 50`` (equal omegas), so
    :meth:`TurnProfile.time_at_angle` is exactly ``phi / omega`` and the
    re-timing reproduces the OLD uniform ``t0 + duration * k/N`` grid
    byte-for-byte — the constant-rate arc TC6 asserts for the default config.
    When the Config makes the snapped stages differ, the final
    ``TURN_SLOW_ZONE_DEG`` degrees sweep at a lower rate, so their sub-keyframes
    are spaced further apart in time — a visible two-stage ramp. The endpoints
    are forced exact so T10 can pin the last sub-keyframe to the snapshot pose.

    :param profile: the resolved :class:`TurnProfile`.
    :param start_pose: ``(x, y, heading_deg)`` at the turn start.
    :param end_pose: ``(x, y, heading_deg)`` at the turn end.
    :param t0: timestamp of the turn-start keyframe.
    :param arc_keyframes: pre-computed pivot-arc sub-keyframes to re-time, or
        ``None`` to synthesize an in-place turn.
    :param signed_theta_deg: the signed commanded rotation, when the caller knows
        it (a raw ``turnDeg`` argument). Used ONLY to disambiguate the exact ±180
        in-place boundary, where the start/end headings alone cannot tell a
        clockwise from a counter-clockwise half-turn. ``None`` keeps the
        shortest-arc default.
    :return: an ordered keyframe list with non-decreasing, profile-derived
        timestamps.
    """
    abs_theta = abs(profile.theta_deg)

    if arc_keyframes is not None:
        n = len(arc_keyframes)
        if n == 0:
            return []
        retimed: List[Dict[str, Any]] = []
        for k, kf in enumerate(arc_keyframes, start=1):
            phi = abs_theta * k / n
            retimed.append(
                _pose_kf(t0 + profile.time_at_angle(phi), kf["x"], kf["y"], kf["h"])
            )
        return _enforce_non_decreasing(retimed)

    # In-place turn: keyframes timed by cumulative swept angle, start -> end.
    x0, y0, h0 = start_pose
    x1, y1, h1 = end_pose
    dh = _shortest_arc(h0, h1)
    # At an exact ±180 delta _shortest_arc canonicalizes to +180 (both directions
    # share the same start/end headings), which would sweep a recorded
    # turnDeg(-180) counter-clockwise. When the caller knows the signed commanded
    # rotation, adopt its sign at the boundary so the interior heading animation
    # turns the way the robot actually did. Only the ±180 boundary is touched —
    # every other magnitude keeps the shortest-arc direction.
    if signed_theta_deg is not None and abs(abs(dh) - 180.0) < 1e-9:
        dh = 180.0 if signed_theta_deg >= 0.0 else -180.0

    sweep = abs_theta if profile.theta_deg else abs(dh)
    n = max(1, ceil(sweep / ARC_STEP_DEG))
    n = min(n, KEYFRAME_COUNT_CEILING - 1)
    n = max(1, n)

    keyframes = [_pose_kf(t0, x0, y0, h0)]
    for k in range(1, n + 1):
        frac = k / n
        x = x0 + (x1 - x0) * frac
        y = y0 + (y1 - y0) * frac
        h = (h0 + dh * frac) % 360.0
        keyframes.append(_pose_kf(t0 + profile.time_at_angle(sweep * frac), x, y, h))
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

    - **cruise**: ``power`` overrides everything (except on the seek verbs — see
      direction sign); else the ``preset`` maps (slow/normal/fast) — default
      NORMAL. For a LINE-follow method the default cruise is
      ``ConfigLine::CRUISE_SPEED``; for ``wallStop`` it is a CONSTANT
      ``WALL_STOP_PUSH_PWM`` push, while ``stopAtLine`` keeps the NORMAL default
      (both match the goto-hop primitive model in ``_primitive_drive_speed``).
    - **direction sign**: on ``driveDist`` ``.setPower`` takes a SIGNED
      ``[-100, 100]`` value and has NO ``setReverse`` — a NEGATIVE power IS the
      reverse mechanism (``Drive.cpp`` drives a negative cruise backwards
      implicitly). So when ``power`` is set on ``driveDist`` the effective
      direction is ``sign(power)`` and the signed power is used directly as the
      cruise (it is NOT ``abs``-ed, and the ``reverse`` flag does NOT
      double-negate it — direction comes through exactly one channel). The
      REVERSIBLE seek verbs (``wallStop`` / ``stopAtLine``) instead store power as
      a MAGNITUDE and carry direction on ``reverse`` separately (firmware
      ``_resolvePower`` + ``cmd.reverse``), so their power is ``abs``-ed and
      ``reverse`` supplies the sign. When ``power`` is unset the direction always
      comes from the ``reverse`` flag, which negates the preset/Config cruise
      magnitude so a reverse hop drives a reverse profile.
    - **start**: default is SIGN-BY-CRUISE —
      ``cruise >= 0 ? +TSHAPE_START_SPEED : -TSHAPE_START_SPEED`` — so a reverse
      hop seeds a REVERSE ramp, not a forward one. An explicit ``start_speed``
      overrides.
    - **end**: default is 0 (brake to rest); ``.setBrake(false)`` (``brake ==
      False``) defaults ``end`` to ``cruise`` (NOT 0), so a no-brake / chained
      move renders no phantom decel-to-zero. An explicit ``end_speed`` overrides
      either default.
    - **profiled**: the recorded ``profiled`` flag (``.tShape()`` -> True,
      ``.flat()`` -> False); absent -> True (the ``driveDist`` builder default),
      EXCEPT ``wallStop`` absent -> False (a constant-push crawl, no ramp).

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
    method_key = method.strip().lower()
    is_line = method_key == "linefollowdist"
    # Seek verbs (wallStop / stopAtLine) are REVERSIBLE: the firmware stores their
    # power as a MAGNITUDE (`_resolvePower`) and applies the `reverse` flag
    # separately — unlike driveDist, whose setPower carries the sign itself and has
    # no setReverse. So a seek verb's power must NOT be treated as signed, and its
    # `reverse` must fold into the cruise sign.
    is_wall_stop = method_key == "wallstop"
    is_seek = is_wall_stop or method_key == "stopatline"

    # --- cruise (signed) ---
    if power is not None and not is_seek:
        # driveDist: `.setPower` carries the SIGNED direction itself ([-100, 100],
        # no setReverse); a negative power is the reverse mechanism for a raw
        # dead-reckon driveDist. Use it directly — do NOT abs() it and do NOT let
        # `reverse` double-negate it (direction comes from exactly one channel;
        # when power is set on a non-seek verb, that channel is its sign).
        cruise = int(power)
    else:
        if power is not None:
            # Reversible seek verb with an explicit setPower: power is a MAGNITUDE;
            # the `reverse` flag below supplies the sign.
            cruise_mag = abs(int(power))
        elif preset is not None:
            cruise_mag = _preset_to_pwm(str(preset), cfg)
        elif is_line:
            cruise_mag = int(cfg.line_cruise)
        elif is_wall_stop:
            # wallStop is a CONSTANT push at WALL_STOP_PUSH_PWM (no ramp) — it does
            # NOT default to the SPEED_NORMAL preset the way stopAtLine does. Matches
            # the goto-hop primitive model (`_primitive_drive_speed`, ACT_WALL_STOP).
            cruise_mag = int(cfg.wall_push_pwm)
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
    if profiled_flag is not None:
        profiled = bool(profiled_flag)
    elif is_wall_stop:
        # wallStop renders as a CONSTANT-push crawl (flat, no ramp), matching the
        # firmware seek and the goto-hop primitive (ACT_WALL_STOP -> profiled=False).
        profiled = False
    else:
        profiled = True

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
    "sample_drive_run",
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
        f"cruiseDeg={turn.cruise_deg:.0f} slowDeg={turn.slow_deg:.0f} "
        f"omegaC={turn.omega_cruise_deg_s:.1f} omegaS={turn.omega_slow_deg_s:.1f}deg/s "
        f"dur={turn.duration_s():.3f}s"
    )
    print(
        f"pivot fast={pivot.fast_pwm} slow={pivot.slow_pwm} "
        f"cruiseDeg={pivot.cruise_deg:.0f} slowDeg={pivot.slow_deg:.0f} "
        f"omegaC={pivot.omega_cruise_deg_s:.1f} omegaS={pivot.omega_slow_deg_s:.1f}deg/s "
        f"dur={pivot.duration_s():.3f}s"
    )


if __name__ == "__main__":
    _selfcheck()
