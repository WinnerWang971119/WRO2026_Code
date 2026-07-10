"""Firmware-faithful float32 mirror of the collinear-run chain speed planner.

This module is the SINGLE Python source of the two-pass junction velocity
planner the CTPS player and the offline sim render for a collinear "chain" of
hops driven as ONE continuous motion (ADR-059, the collinear-run
continuous-motion plan). :func:`resolve_chain_speeds` is a pure float32 mirror
of ``resolveChainSpeeds`` in ``WRO2026/src/drive/ChainSpeedPolicy.h`` and is
pinned bit-for-bit to the firmware by a cross-language golden-vector test (T5 /
AC6 / TC1): it reproduces every vector in
``tests/fixtures/chain_speed_goldens.json`` exactly.

Float32 emulation (the parity contract)
---------------------------------------
The C++ policy carries every node speed as a 32-bit ``float`` through BOTH the
forward and backward passes (the ``accel * hopDist`` products, the additions,
and the ``min`` compares are all float32), then truncates each node speed to
``int16_t`` toward zero ONLY at the output boundary before multiplying the run
sign in. A naive ``float64`` port diverges in the low bits and fails the
golden-vector test. So every real intermediate here rounds through
:class:`numpy.float32` via :func:`~tools.path_planner.ctps.profile._f32`, and
every ``int16_t`` cast truncates toward zero via
:func:`~tools.path_planner.ctps.profile._trunc_i16` — the same two primitives
``profile.py`` pins its T-shape kernel to (reused directly, NOT re-implemented,
so the two mirrors cannot drift in their float32 discipline).

The algorithm (mirrors ``ChainSpeedPolicy.h`` operation-for-operation — do NOT
algebraically reassociate; the low bits must match the host dump)
------------------------------------------------------------------
Node speeds ``v[0..hopCount]`` are resolved in the MAGNITUDE domain; the sign of
every output comes from ``cruise_pwm`` (a reverse-cruise run has a negative
``cruise_pwm`` and yields negative ``startPwm`` / ``endPwm``):

1. **Seed.** ``v[0] = clamp(|entry|, pwmMin, |cruise|)``; every interior/target
   node ``v[1..hopCount]`` seeds to ``|cruise|``.
2. **Forward pass (accel limit).** For ``k = 0..hopCount-1``:
   ``v[k+1] = min(v[k+1], v[k] + accel*hopDist[k])``.
3. **Backward pass (decel to floor, chained).** Seed the terminal node
   ``v[hopCount] = min(v[hopCount], pwmMin)`` (the final hop is the ordinary
   cruise->floor->brake stop), then walk back ``k = hopCount-1 .. 0``:
   ``v[k] = min(v[k], v[k+1] + accel*hopDist[k])``. Chaining through the
   ALREADY-reduced downstream node ``v[k+1]`` (not a per-hop
   ``pwmMin + accel*distToEnd`` closed form) keeps every commanded decel
   ``<= accel`` and pulls the shed into the PRIOR hop for a short final hop.
4. **Map to hops.** For ``k = 0..hopCount-2``:
   ``out[k] = {sign*trunc(v[k]), sign*trunc(v[k+1]), brake=False}``. Final hop:
   ``out[hopCount-1] = {sign*trunc(v[hopCount-1]), 0, brake=True}``.

Degenerate inputs (Error Handling): ``hop_count < 2`` is NOT a chain — the
function returns an EMPTY list, leaving the caller to take the ordinary per-hop
path (TC1's single-hop rejection). A zero/negative ``hop_dist_cm[k]`` floors to
0; ``accel <= 0`` clamps to :data:`CHAIN_ACCEL_EPS` (``1e-6``, matching the C++
``CHAIN_ACCEL_EPS`` and ``profile.py``'s ``1e-6`` defensive floor). Nothing here
divides, so there is no divide-by-zero to guard beyond the accel floor.

Public API (see the plan Contracts, 2026-07-10 collinear-run spec)
------------------------------------------------------------------
- :class:`ChainSpeedConfig` — the four Config knobs the planner needs (POD).
- :func:`load_chain_speed_config` — parse the knobs from the firmware headers,
  the SAME ``Config::*`` constants the host golden dumper
  (``chain_speed_dump.cpp``) builds its ``ChainSpeedConfig`` from.
- :class:`ChainHopSpeed` — one resolved hop's signed ``(start_pwm, end_pwm)`` +
  ``brake`` flag.
- :func:`resolve_chain_speeds` — the pure float32 mirror.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

from tools.path_planner.config_parse import (
    DEFAULT_CONFIG_H_PATH,
    parse_config_float,
    parse_config_int,
)
from tools.path_planner.ctps.profile import _f32, _trunc_i16

# ==============================================================================
# Defensive accel floor (mirrors ChainSpeedPolicy.h's CHAIN_ACCEL_EPS)
# ==============================================================================

#: Smallest positive accel slope used when the caller passes ``accel <= 0``.
#:
#: The firmware bails on ``accel <= 0`` before the policy is reached, but a
#: standalone caller (or a mis-configured mirror) must not divide the
#: accel-limited passes into a zero slope — so the policy clamps up to this tiny
#: positive value, matching the C++ ``CHAIN_ACCEL_EPS = 1e-6f`` and
#: ``profile.py``'s ``1e-6`` defensive floor. There is no divide in the mid-pass
#: math, so this only keeps the accel*dist products non-degenerate.
CHAIN_ACCEL_EPS: float = 1e-6


# ==============================================================================
# Config knobs — the four values chain_speed_dump.cpp builds ChainSpeedConfig from
# ==============================================================================


@dataclass(frozen=True)
class ChainSpeedConfig:
    """The Config knobs the chain planner needs — mirror of ``ChainSpeedConfig``.

    A POD that mirrors the C++ ``ChainSpeedConfig`` struct field-for-field. The
    host golden dumper (``chain_speed_dump.cpp``) builds its
    ``ChainSpeedConfig`` from ``Config::PWM_MIN_MAGNITUDE`` /
    ``Config::PWM_MAX`` / ``Config::TSHAPE_ACCEL_PWM_PER_CM`` /
    ``Config::TSHAPE_MIN_MOVE_CM``; :func:`load_chain_speed_config` parses those
    SAME four constants out of the firmware headers so the mirror plans against
    the identical config the goldens were dumped with.

    All PWM values are signed magnitudes in the DriveDC ``[-100, 100]`` domain;
    ``accel_pwm_per_cm`` is PWM-per-cm; ``min_move_cm`` is cm.
    """

    pwm_min: int          # PWM_MIN_MAGNITUDE (motor deadband floor magnitude, 50)
    pwm_max: int          # PWM_MAX (saturation cap magnitude, 95; defensive)
    accel_pwm_per_cm: float  # TSHAPE_ACCEL_PWM_PER_CM (ramp slope, 5.0)
    min_move_cm: float    # TSHAPE_MIN_MOVE_CM (informational; caller gates on it)


def load_chain_speed_config(config_path: Path = DEFAULT_CONFIG_H_PATH) -> ChainSpeedConfig:
    """Parse the four chain-planner Config knobs, fail-loud (issue #44).

    Reads the SAME ``Config::*`` constants the host golden dumper
    (``chain_speed_dump.cpp``) builds its ``ChainSpeedConfig`` from, via the ONE
    place header parsing lives (:mod:`tools.path_planner.config_parse`), so the
    Python mirror plans against the identical config the committed goldens were
    dumped with. A missing/unparseable constant raises rather than falling back
    to a stale literal.

    The constants are distributed across two headers:

    - ``ConfigGlobal.h``: ``PWM_MIN_MAGNITUDE`` / ``PWM_MAX`` (``int16_t``).
    - ``ConfigDrive.h`` (``config_path``): ``TSHAPE_ACCEL_PWM_PER_CM`` /
      ``TSHAPE_MIN_MOVE_CM`` (``float``).

    The sibling ``ConfigGlobal.h`` is resolved relative to ``config_path``'s
    directory so a test that copies a perturbed header set into a temp dir reads
    a self-consistent pair, while the default reads the real firmware headers.

    :param config_path: the ``ConfigDrive.h`` holding the ``TSHAPE_*`` knobs;
        defaults to the real header resolved relative to this module
        (cwd-independent). Its parent directory must also hold ``ConfigGlobal.h``.
    :return: a fully-populated :class:`ChainSpeedConfig`.
    :raises OSError: when ``config_path`` (or ``ConfigGlobal.h``) is unreadable.
    :raises ValueError: when a required constant is missing or unparseable.
    """
    global_h = config_path.parent / "ConfigGlobal.h"
    return ChainSpeedConfig(
        pwm_min=parse_config_int("PWM_MIN_MAGNITUDE", global_h),
        pwm_max=parse_config_int("PWM_MAX", global_h),
        accel_pwm_per_cm=parse_config_float("TSHAPE_ACCEL_PWM_PER_CM", config_path),
        min_move_cm=parse_config_float("TSHAPE_MIN_MOVE_CM", config_path),
    )


# ==============================================================================
# One resolved hop — mirror of the C++ ChainHopSpeed struct
# ==============================================================================


@dataclass(frozen=True)
class ChainHopSpeed:
    """One resolved hop of a chain — mirror of the C++ ``ChainHopSpeed`` struct.

    The signed start/end PWM setpoints plus the brake flag. ``brake`` is ``True``
    ONLY on the final hop (``out[hop_count-1]``), whose ``end_pwm`` is ``0`` — the
    ordinary cruise->floor->brake stop. Every mid-hop has ``brake=False`` and a
    non-zero ``end_pwm`` so the chassis passes through the node without braking to
    rest.
    """

    start_pwm: int  # Signed PWM commanded at the hop start (sign = run direction).
    end_pwm: int    # Signed PWM commanded at the hop end; 0 on the final hop.
    brake: bool     # True only on the final hop.


# ==============================================================================
# The two-pass planner — pure float32 mirror of resolveChainSpeeds
# ==============================================================================


def resolve_chain_speeds(
    hop_dist_cm: List[float],
    hop_count: int,
    cruise_pwm: int,
    entry_pwm: int,
    cfg: Optional[ChainSpeedConfig] = None,
) -> List[ChainHopSpeed]:
    """Resolve a collinear run of ``hop_count`` hops — mirror of ``resolveChainSpeeds``.

    PURE float32 mirror of the C++ ``resolveChainSpeeds`` (see the module
    docstring for the full algorithm). Every node-speed intermediate is computed
    in :class:`numpy.float32` via :func:`_f32`, and each node speed is truncated
    to ``int16_t`` toward zero via :func:`_trunc_i16` ONLY at the output
    boundary, then the run sign is multiplied in. This reproduces the committed
    golden vectors bit-for-bit (AC6 / TC1).

    :param hop_dist_cm: per-hop distances (cm); negative/zero entries floor to 0.
        Must have at least ``hop_count`` entries.
    :param hop_count: number of hops in the run (``>= 2`` for a chain; ``< 2``
        returns an empty list, the caller's ordinary per-hop path).
    :param cruise_pwm: signed run cruise PWM (its sign carries the run direction
        into every output; a cruise of exactly 0 is treated as forward).
    :param entry_pwm: signed PWM the chassis is physically at when the run begins
        (the captured cruise after a pass-through, else +/-TSHAPE_START_SPEED on a
        cold start).
    :param cfg: the ``pwm_min`` / ``pwm_max`` / ``accel`` knobs; defaults to
        :func:`load_chain_speed_config` (the real firmware headers).
    :return: a list of ``hop_count`` :class:`ChainHopSpeed` for ``hop_count >= 2``;
        an EMPTY list for ``hop_count < 2`` (not a chain).
    """
    # hop_count < 2 is not a chain — the caller uses the ordinary per-hop path.
    # Return empty (mirrors the C++ leaving `out` untouched; TC1 single-hop).
    if hop_count < 2:
        return []

    cfg = cfg or load_chain_speed_config()

    # Run sign comes from the cruise PWM; the two-pass math is done in the
    # magnitude domain. A cruise of exactly 0 has no direction, so treat it as
    # forward (+1) — a degenerate input the caller never produces, guarded only
    # so the sign is defined (mirrors the C++ `sign` computation).
    sign = -1 if cruise_pwm < 0 else 1

    # Integer magnitudes promoted to float once, up front — the clamp bounds and
    # the entry seed enter the float32 domain here (BEHAVIOR-IDENTITY contract).
    cruise_mag_i = -int(cruise_pwm) if cruise_pwm < 0 else int(cruise_pwm)
    entry_mag_i = -int(entry_pwm) if entry_pwm < 0 else int(entry_pwm)
    cruise_mag = _f32(float(cruise_mag_i))
    floor_mag = _f32(float(cfg.pwm_min))

    # Defensive accel floor: never divide the accel-limited passes into a
    # zero/negative slope (mirrors the C++ ACCEL_EPS clamp).
    accel = _f32(cfg.accel_pwm_per_cm) if cfg.accel_pwm_per_cm > 0.0 else _f32(CHAIN_ACCEL_EPS)

    node_count = hop_count + 1
    hops = node_count - 1

    # ---- 1. Seed. --------------------------------------------------------------
    # v[0] = clamp(|entry|, pwmMin, |cruise|); interior/target nodes seed |cruise|.
    v0 = _f32(float(entry_mag_i))
    if v0 < floor_mag:
        v0 = floor_mag
    if v0 > cruise_mag:
        v0 = cruise_mag
    v: List[float] = [v0] + [cruise_mag] * (node_count - 1)

    # ---- 2. Forward pass (accel limit): v[k+1] = min(v[k+1], v[k] + accel*d). ---
    for k in range(hops):
        d = _f32(hop_dist_cm[k])
        if d < 0.0:
            d = _f32(0.0)
        reachable = _f32(v[k] + _f32(accel * d))
        if reachable < v[k + 1]:
            v[k + 1] = reachable

    # ---- 3. Backward pass (decel to floor): terminal -> pwmMin, chain v[k+1]. ---
    if floor_mag < v[hops]:
        v[hops] = floor_mag
    for k in range(hops - 1, -1, -1):
        d = _f32(hop_dist_cm[k])
        if d < 0.0:
            d = _f32(0.0)
        sheddable = _f32(v[k + 1] + _f32(accel * d))
        if sheddable < v[k]:
            v[k] = sheddable

    # ---- 5. Map node speeds to per-hop signed (start_pwm, end_pwm). Truncate
    #         toward zero at the output boundary, then multiply the run sign in. --
    out: List[ChainHopSpeed] = []
    for k in range(hops):
        start_mag = _trunc_i16(v[k])
        start_pwm = sign * start_mag
        if k + 1 == hops:
            # Final hop: the ordinary cruise->floor->brake stop (end 0, brake).
            out.append(ChainHopSpeed(start_pwm=start_pwm, end_pwm=0, brake=True))
        else:
            end_mag = _trunc_i16(v[k + 1])
            end_pwm = sign * end_mag
            out.append(ChainHopSpeed(start_pwm=start_pwm, end_pwm=end_pwm, brake=False))

    return out
