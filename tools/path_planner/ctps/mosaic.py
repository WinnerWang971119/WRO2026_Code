"""Mosaic roll, geometry resolve, and the single-source colour contract (CTPS).

This is the leaf module for the CTPS "randomized vision" feature (ADR-060): the
**one place** the mosaic colour vocabulary (:data:`COLOR_NAMES` /
:data:`COLOR_LETTER` / :data:`COLOR_HEX`, plus the :data:`AIR_NAME` /
:data:`AIR_LETTER` empty-cell sentinel and the :func:`color_to_letter` wire
join), the seeded roll (:func:`roll_mosaic`), the world-frame cell geometry
(:func:`resolve_cells`),
the geometry fingerprint (:func:`geom_hash`), and the pure schema validators
(:func:`validate_mosaic_config` / :func:`validate_vision_cone`) live. Every
downstream consumer reads from here and never re-hardcodes any of it:

* :mod:`tools.path_planner.ctps.layout` (T2) reuses the validators (wrapping the
  raised :class:`ValueError` as a ``LayoutError`` with a key path) and the
  colour vocabulary — it never re-implements a mosaic rule.
* :mod:`tools.path_planner.ctps.trace_loader` (T8) whitelists the trace mosaic
  block against :data:`COLOR_NAMES`.
* The orchestrator + ``player_html`` (T10) roll via :func:`roll_mosaic`, resolve
  cells via :func:`resolve_cells`, and fingerprint via :func:`geom_hash`.
* ``test_ctps_color_enum_parity`` (T3) regex-parses ``enum class Color`` out of
  ``WRO2026/config/ConfigVision.h`` and asserts it equals :data:`COLOR_NAMES`
  in name **and** order.

Colour contract (must match ``Config::Color`` minus ``UNKNOWN``):
``enum class Color : uint8_t { UNKNOWN=0, YELLOW, BLUE, GREEN, WHITE, RED };`` ⇒
:data:`COLOR_NAMES` ``= ("YELLOW", "BLUE", "GREEN", "WHITE", "RED")`` in exactly
that order. Drift fails CI via the T3 header-parse guard. The ``UNKNOWN=0``
enumerant is exposed separately as the AIR (empty-cell) sentinel
(:data:`AIR_NAME` ``= "UNKNOWN"``, :data:`AIR_LETTER` ``= "A"``); it is NOT in
:data:`COLOR_NAMES`, so the parity guard is unaffected (ADR-063).

Cell ordering / geometry (authoritative — the same index ``i`` is the position
in ``cells[]``, the ``counts``-fill order, the trace ``mosaic.cells[]``,
``MOSAIC.cells[].index``, and the firmware ``Vision.getColor(i)`` index)::

    i        = row * cols + col          # row 0 at +y "top", col 0 at left
    local_x  = (col - (cols - 1) / 2) * (cell_w_cm + col_gap_cm)
    local_y  = ((rows - 1) / 2 - row)   * (cell_h_cm + row_gap_cm)
    world    = (center_x_cm, center_y_cm) + Rot(heading_deg) . (local_x, local_y)

``Rot(theta)`` is the standard CCW rotation by ``heading_deg`` (degrees).

Frames: the ``mosaic`` block is WORLD-frame (centimetres); the ``robot.vision``
cone is robot-LOCAL (``+x`` forward, ``+y`` left) — this module validates the
cone but does not consume its geometry (the JS ``visibleCells`` predicate does,
T11).

This module has NO dependency on the heavy CTPS modules — it is imported BY
``layout.py`` (and others), never the reverse, so there is no circular import.
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass, field as dc_field
from typing import Any, Dict, List, Optional, Tuple

__all__ = [
    "COLOR_NAMES",
    "COLOR_LETTER",
    "COLOR_HEX",
    "AIR_NAME",
    "AIR_LETTER",
    "color_to_letter",
    "MODE_DIRECTED",
    "MODE_RANDOM",
    "CELL_COUNT",
    "MosaicConfig",
    "roll_mosaic",
    "resolve_cells",
    "geom_hash",
    "validate_mosaic_config",
    "validate_vision_cone",
]


# ---------------------------------------------------------------------------
# Colour contract (SINGLE SOURCE — never re-hardcode any of these elsewhere)
# ---------------------------------------------------------------------------

#: Mosaic colour names, in ``Config::Color`` declaration order with ``UNKNOWN``
#: stripped (``ConfigVision.h``: ``UNKNOWN=0, YELLOW, BLUE, GREEN, WHITE, RED``).
#: The T3 header-parse guard asserts this tuple equals the parsed enum body
#: (name + order); drift fails CI.
COLOR_NAMES: Tuple[str, ...] = ("YELLOW", "BLUE", "GREEN", "WHITE", "RED")

#: Single-letter code per colour — the ``ctps_dump --mosaic <12 letters>`` wire
#: alphabet (the C++ mock parses letter -> ``Config::Color`` against the real
#: enum). Exactly one letter per :data:`COLOR_NAMES` entry; the AIR sentinel
#: carries its own :data:`AIR_LETTER` (join via :func:`color_to_letter`).
COLOR_LETTER: Dict[str, str] = {
    "YELLOW": "Y",
    "BLUE": "B",
    "GREEN": "G",
    "WHITE": "W",
    "RED": "R",
}

#: Display-only swatch hex per colour (emitted as ``MOSAIC_META.hex`` for the
#: player / editor). NOT a firmware value — purely the on-screen colour.
COLOR_HEX: Dict[str, str] = {
    "YELLOW": "#F2C200",
    "BLUE": "#2563EB",
    "GREEN": "#16A34A",
    "WHITE": "#F5F5F5",
    "RED": "#DC2626",
}

#: The set of valid colour names (for O(1) membership tests in validators).
_COLOR_NAME_SET: frozenset[str] = frozenset(COLOR_NAMES)

#: The "air" (empty) cell — an unlit mosaic cell with no block present. It maps
#: to the firmware ``Config::Color::UNKNOWN`` enumerant (declaration value 0),
#: whose canonical name string is ``"UNKNOWN"`` (exactly what the C++
#: ``ctps_record`` emits for that enum). AIR is deliberately kept OUT of
#: :data:`COLOR_NAMES` / :data:`COLOR_HEX` — it is not a palette colour: the
#: header-parse enum-parity guard strips ``UNKNOWN`` from the enum body,
#: ``MOSAIC_META`` carries only the five real colours, and the player renders an
#: AIR cell through its existing unknown-colour outline fallback (no synthesized
#: fill). AIR is produced ONLY by :func:`roll_mosaic` when random-mode counts
#: under-fill the null cells; it is never an authorable/pinnable value (ADR-063).
AIR_NAME: str = "UNKNOWN"

#: Wire letter for an AIR cell on the ``ctps_dump --mosaic <letters>`` argv. The
#: C++ ``letterToColor`` maps ``'A' -> Config::Color::UNKNOWN`` (the mirror of
#: this constant). Distinct from every :data:`COLOR_LETTER` value.
AIR_LETTER: str = "A"


def color_to_letter(name: str) -> str:
    """Map a rolled cell colour name to its single ``--mosaic`` wire letter.

    The single source of the wire alphabet (ADR-060 keeps it here, never
    re-hardcoded): the five palette colours resolve through
    :data:`COLOR_LETTER`, and the AIR sentinel (:data:`AIR_NAME`) resolves to
    :data:`AIR_LETTER`. The orchestrator builds the ``--mosaic`` argv value by
    joining this over a rolled cell list, so an AIR cell (from an under-filled
    random roll) travels the wire as :data:`AIR_LETTER` and the C++ dump injects
    it as ``Config::Color::UNKNOWN``.

    :param name: a rolled cell colour name — a :data:`COLOR_NAMES` entry or
        :data:`AIR_NAME`.
    :return: the single wire letter for ``name``.
    :raises KeyError: if ``name`` is neither a palette colour nor
        :data:`AIR_NAME`.
    """
    if name == AIR_NAME:
        return AIR_LETTER
    return COLOR_LETTER[name]

#: Roll modes — exact strings, shared with the schema / editor.
MODE_DIRECTED: str = "directed"
MODE_RANDOM: str = "random"
_ALLOWED_MODES: frozenset[str] = frozenset({MODE_DIRECTED, MODE_RANDOM})

#: The fixed mosaic cell count (``rows * cols`` MUST equal this — the WRO 2026
#: 4x3 grid, pinned to ``Config::VISION_CELL_COUNT``).
CELL_COUNT: int = 12

#: The INTEGER geometry keys (grid dimensions) — hashed as bare ints.
_GEOM_INT_KEYS: Tuple[str, ...] = ("rows", "cols")

#: The FLOAT geometry keys (cm / degree measurements) — hashed as a fixed
#: ``".6g"`` string so ``5`` and ``5.0`` (and ``-0.0`` / ``0.0``) fingerprint
#: identically regardless of the incoming numeric type.
_GEOM_FLOAT_KEYS: Tuple[str, ...] = (
    "cell_w_cm",
    "cell_h_cm",
    "row_gap_cm",
    "col_gap_cm",
    "center_x_cm",
    "center_y_cm",
    "heading_deg",
)

#: The full geometry key set hashed by :func:`geom_hash` and required on a
#: geometry object (order-independent — the fingerprint sorts keys).
_GEOM_KEYS: Tuple[str, ...] = _GEOM_INT_KEYS + _GEOM_FLOAT_KEYS


# ---------------------------------------------------------------------------
# Numeric guards (mirror layout.py's convention — reject bool / NaN / Inf)
# ---------------------------------------------------------------------------


def _is_finite_number(value: Any) -> bool:
    """Return True for a finite ``int``/``float`` that is not a ``bool``.

    A JSON ``true``/``false`` decodes to Python ``bool`` (a subclass of
    ``int``), so a stray boolean would otherwise silently pass as ``1``/``0``.
    NaN / Infinity are also rejected — they slip past every downstream range
    gate (``<= 0`` is False for NaN) with no diagnostic. Mirrors
    :func:`tools.path_planner.ctps.layout._is_finite_number`.

    :param value: the decoded JSON value to test.
    :return: ``True`` iff ``value`` is a finite, non-bool ``int``/``float``.
    """
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )


def _is_nonneg_int(value: Any) -> bool:
    """Return True for a non-negative ``int`` that is not a ``bool``.

    Used for the mosaic ``seed`` and the integer grid dimensions. Rejects
    ``-1`` (negative), ``1.5`` (a float — even ``1.0`` is rejected: a seed must
    be an ``int``), and ``True`` (a bool).

    :param value: the decoded JSON value to test.
    :return: ``True`` iff ``value`` is an ``int >= 0`` and not a ``bool``.
    """
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0


# ---------------------------------------------------------------------------
# Mosaic configuration dataclass
# ---------------------------------------------------------------------------


@dataclass
class MosaicConfig:
    """A parsed ``mosaic`` block — grid geometry, palette, pins, counts, mode.

    Mirrors the ``layout.json`` ``mosaic`` block (Data Model). Carries both the
    ROLL inputs (``mode`` / ``palette`` / ``cells`` / ``counts`` / ``seed``) and
    the GEOMETRY inputs (``center_*`` / ``heading_deg`` / ``rows`` / ``cols`` /
    ``cell_*`` / ``*_gap_cm``) so a single object drives both
    :func:`roll_mosaic` and :func:`resolve_cells`.

    Construct from a decoded ``mosaic`` dict via :meth:`from_dict` (which does
    NOT validate — call :func:`validate_mosaic_config` on the raw dict first, as
    ``layout.py`` does) and round-trip back via :meth:`to_dict`.

    :param mode: ``"random"`` or ``"directed"``.
    :param center_x_cm: mosaic seat centre X (world frame, cm).
    :param center_y_cm: mosaic seat centre Y (world frame, cm).
    :param heading_deg: mosaic orientation (CCW degrees; ``0`` = grid rows run
        along ``+x``).
    :param rows: grid row count; ``rows * cols`` MUST equal :data:`CELL_COUNT`.
    :param cols: grid column count.
    :param cell_w_cm: single cell width (cm, along local ``x``); ``> 0``.
    :param cell_h_cm: single cell height (cm, along local ``y``); ``> 0``.
    :param row_gap_cm: uniform gap between rows (cm); ``>= 0``.
    :param col_gap_cm: uniform gap between columns (cm); ``>= 0``.
    :param palette: allowed colour names (a subset of :data:`COLOR_NAMES`).
    :param cells: exactly :data:`CELL_COUNT` entries — each ``None`` (rolled) or
        a pinned palette colour name.
    :param counts: random-mode quota per colour for the NULL cells. The values
        may sum to any non-negative number (ADR-063): an under-fill leaves the
        leftover null cells as air, an over-fill keeps a random subset. Empty /
        ignored in directed mode.
    :param seed: fixed non-negative ``int`` seed, or ``None`` (fresh per
        invocation — the caller resolves it).
    """

    mode: str
    center_x_cm: float
    center_y_cm: float
    heading_deg: float
    rows: int
    cols: int
    cell_w_cm: float
    cell_h_cm: float
    row_gap_cm: float
    col_gap_cm: float
    palette: List[str] = dc_field(default_factory=list)
    cells: List[Optional[str]] = dc_field(default_factory=list)
    counts: Dict[str, int] = dc_field(default_factory=dict)
    seed: Optional[int] = None

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "MosaicConfig":
        """Build a :class:`MosaicConfig` from a decoded ``mosaic`` dict.

        Does NOT validate — the caller is expected to have run
        :func:`validate_mosaic_config` on ``data`` first (``layout.py`` does),
        so this is a plain field-lift. Numeric geometry is coerced to ``float``
        / ``int`` and ``counts`` values to ``int`` so the object is uniformly
        typed regardless of whether the JSON carried ``5`` or ``5.0``.

        :param data: an already-validated ``mosaic`` block dict.
        :return: the constructed :class:`MosaicConfig`.
        """
        return cls(
            mode=str(data["mode"]),
            center_x_cm=float(data["center_x_cm"]),
            center_y_cm=float(data["center_y_cm"]),
            heading_deg=float(data["heading_deg"]),
            rows=int(data["rows"]),
            cols=int(data["cols"]),
            cell_w_cm=float(data["cell_w_cm"]),
            cell_h_cm=float(data["cell_h_cm"]),
            row_gap_cm=float(data["row_gap_cm"]),
            col_gap_cm=float(data["col_gap_cm"]),
            palette=[str(name) for name in data["palette"]],
            cells=[None if c is None else str(c) for c in data["cells"]],
            counts={str(k): int(v) for k, v in data.get("counts", {}).items()},
            seed=None if data.get("seed") is None else int(data["seed"]),
        )

    def to_dict(self) -> Dict[str, Any]:
        """Return the schema dict for this mosaic block.

        Round-trips (through :func:`json.dumps`/``json.loads``) with the dict
        :meth:`from_dict` produced this object from.

        :return: the ``mosaic`` block dict (schema_version 1 shape).
        """
        return {
            "mode": self.mode,
            "center_x_cm": self.center_x_cm,
            "center_y_cm": self.center_y_cm,
            "heading_deg": self.heading_deg,
            "rows": self.rows,
            "cols": self.cols,
            "cell_w_cm": self.cell_w_cm,
            "cell_h_cm": self.cell_h_cm,
            "row_gap_cm": self.row_gap_cm,
            "col_gap_cm": self.col_gap_cm,
            "palette": list(self.palette),
            "cells": list(self.cells),
            "counts": dict(self.counts),
            "seed": self.seed,
        }

    def geom_dict(self) -> Dict[str, Any]:
        """Return only the GEOMETRY sub-dict (the :func:`geom_hash` inputs).

        The subset of :meth:`to_dict` that :func:`resolve_cells` and
        :func:`geom_hash` read — grid dimensions, cell sizes, gaps, seat centre,
        and heading. Excludes the roll inputs (palette / cells / counts / mode /
        seed), so re-rolling the SAME geometry hashes identically.

        :return: a dict carrying exactly :data:`_GEOM_KEYS`.
        """
        return {
            "rows": self.rows,
            "cols": self.cols,
            "cell_w_cm": self.cell_w_cm,
            "cell_h_cm": self.cell_h_cm,
            "row_gap_cm": self.row_gap_cm,
            "col_gap_cm": self.col_gap_cm,
            "center_x_cm": self.center_x_cm,
            "center_y_cm": self.center_y_cm,
            "heading_deg": self.heading_deg,
        }


# ---------------------------------------------------------------------------
# Roll (AC2 / TC1-TC4)
# ---------------------------------------------------------------------------


def roll_mosaic(cfg: MosaicConfig, rng: "Any") -> List[str]:
    """Roll the 12 mosaic cell colours for a config, by cell index.

    Deterministic for a fixed ``rng`` (AC2 / TC1): all randomness is drawn from
    the passed :class:`random.Random` — seed determinism is the CALLER's
    concern (the orchestrator resolves ``--seed`` / ``mosaic.seed`` / fresh
    entropy and constructs the ``Random``).

    * **directed** mode: the output is the pinned :attr:`MosaicConfig.cells`
      verbatim (every cell is non-``None`` in directed mode); ``rng`` / seed are
      irrelevant.
    * **random** mode: every pinned (non-``None``) cell keeps its colour; the
      ``None`` cells are filled — in ASCENDING cell-index order — from the
      :attr:`MosaicConfig.counts` multiset shuffled by ``rng``. The pool is the
      counts expanded to one entry per unit (e.g. ``{Y:3, B:3, G:4}`` -> 10
      entries). ``sum(counts)`` need NOT equal the null-cell count (ADR-063):

      * **exact fill** (``sum == #nulls``): the pool is ``rng.shuffle``-d, then
        dealt into the null cells left-to-right (byte-identical to pre-ADR-063).
      * **under-fill** (``sum < #nulls``): the pool is padded with
        :data:`AIR_NAME` up to ``#nulls`` BEFORE the shuffle, so the leftover
        cells become empty ("air") and the air lands at random positions among
        the null cells (not always the tail).
      * **over-fill** (``sum > #nulls``): the pool is shuffled and only the
        first ``#nulls`` units are dealt — the excess is dropped at random, so
        the kept subset is a uniform random sample of the counts multiset.

    Assumes ``cfg`` already passed :func:`validate_mosaic_config` (so
    ``len(cells) == CELL_COUNT``, directed has no null cell, and random's
    ``counts`` keys/values are palette colours / non-negative ints); it does not
    re-validate.

    :param cfg: the mosaic configuration (mode + cells + counts).
    :param rng: a :class:`random.Random` instance (explicitly passed; the seed
        lives with the caller).
    :return: a list of exactly :data:`CELL_COUNT` colour-name strings, indexed
        by cell (``i = row * cols + col``).
    :raises ValueError: if ``cfg.mode`` is neither ``directed`` nor ``random``
        (a defensively-guarded unreachable path for a validated config).
    """
    if cfg.mode == MODE_DIRECTED:
        # Directed: pins verbatim (validation guarantees no None). Copy so the
        # caller can never mutate the config's list through the result.
        return [str(color) for color in cfg.cells]

    if cfg.mode != MODE_RANDOM:
        raise ValueError(
            f"roll_mosaic: unknown mode {cfg.mode!r}; "
            f"expected one of {sorted(_ALLOWED_MODES)!r}."
        )

    # Random: build the null-cell pool from the counts multiset, then deal it
    # into the null cells in ascending index order. Counts are iterated in
    # COLOR_NAMES order so the unshuffled pool is deterministic before the
    # shuffle (the shuffle is the only entropy).
    null_count = sum(1 for cell in cfg.cells if cell is None)
    pool: List[str] = []
    for name in COLOR_NAMES:
        pool.extend([name] * int(cfg.counts.get(name, 0)))

    # Under-fill: pad the pool with AIR so the leftover null cells become empty.
    # Air is padded BEFORE the shuffle so it lands at random positions among the
    # null cells, not always at the tail.
    if len(pool) < null_count:
        pool.extend([AIR_NAME] * (null_count - len(pool)))

    rng.shuffle(pool)

    # Over-fill: len(pool) > null_count, so dealing only null_count entries drops
    # the excess at random (the shuffle already randomised which units survive).
    result: List[str] = []
    pool_iter = iter(pool)
    for pinned in cfg.cells:
        if pinned is None:
            result.append(next(pool_iter))
        else:
            result.append(str(pinned))
    return result


# ---------------------------------------------------------------------------
# Geometry resolve (AC5 / TC5)
# ---------------------------------------------------------------------------


def resolve_cells(
    geom: "Any", color_names: List[str]
) -> List[Dict[str, Any]]:
    """Resolve the 12 WORLD-frame cell centres + colours for a mosaic.

    Computes the load-bearing cell-index -> world-position mapping the player
    draws and the firmware ``Vision.getColor(i)`` semantics mirror. This is the
    AUTHORITATIVE geometry (AC5); the editor's JS ``mosaicCellCenters`` is a
    non-load-bearing preview mirror validated against the same pinned vectors.

    Formula (per cell, from the Data Model)::

        i        = row * cols + col          # row 0 at +y "top"
        local_x  = (col - (cols - 1) / 2) * (cell_w_cm + col_gap_cm)
        local_y  = ((rows - 1) / 2 - row)   * (cell_h_cm + row_gap_cm)
        world    = (center_x_cm, center_y_cm) + Rot(heading_deg) . (local_x, local_y)

    where ``Rot(theta) . (x, y) = (x cos t - y sin t, x sin t + y cos t)`` is
    the standard CCW rotation by ``heading_deg`` degrees.

    :param geom: an object exposing the geometry attributes (a
        :class:`MosaicConfig`, or any object / namespace carrying ``rows``,
        ``cols``, ``cell_w_cm``, ``cell_h_cm``, ``row_gap_cm``, ``col_gap_cm``,
        ``center_x_cm``, ``center_y_cm``, ``heading_deg``).
    :param color_names: the rolled colour name per cell (length
        :data:`CELL_COUNT`), indexed by ``i`` — typically :func:`roll_mosaic`'s
        output or the trace ``mosaic.cells``.
    :return: a list of exactly :data:`CELL_COUNT` dicts
        ``{"index": int, "x_cm": float, "y_cm": float, "color": str}``, in
        ascending index order.
    :raises ValueError: if ``color_names`` length does not equal ``rows * cols``.
    """
    rows = int(geom.rows)
    cols = int(geom.cols)
    cell_w_cm = float(geom.cell_w_cm)
    cell_h_cm = float(geom.cell_h_cm)
    row_gap_cm = float(geom.row_gap_cm)
    col_gap_cm = float(geom.col_gap_cm)
    center_x_cm = float(geom.center_x_cm)
    center_y_cm = float(geom.center_y_cm)
    heading_deg = float(geom.heading_deg)

    expected = rows * cols
    if len(color_names) != expected:
        raise ValueError(
            f"resolve_cells: color_names has {len(color_names)} entries; "
            f"expected rows*cols == {expected}."
        )

    theta = math.radians(heading_deg)
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)

    col_pitch = cell_w_cm + col_gap_cm
    row_pitch = cell_h_cm + row_gap_cm
    col_mid = (cols - 1) / 2.0
    row_mid = (rows - 1) / 2.0

    cells: List[Dict[str, Any]] = []
    for row in range(rows):
        for col in range(cols):
            index = row * cols + col
            local_x = (col - col_mid) * col_pitch
            local_y = (row_mid - row) * row_pitch  # row 0 at +y "top"
            world_x = center_x_cm + (local_x * cos_t - local_y * sin_t)
            world_y = center_y_cm + (local_x * sin_t + local_y * cos_t)
            cells.append(
                {
                    "index": index,
                    "x_cm": world_x,
                    "y_cm": world_y,
                    "color": str(color_names[index]),
                }
            )
    return cells


# ---------------------------------------------------------------------------
# Geometry fingerprint (platform-stable — Windows == Linux/macOS)
# ---------------------------------------------------------------------------


def _canonical_float(value: Any) -> str:
    """Format one FLOAT geometry value deterministically for hashing.

    Coerces to ``float`` and renders with a fixed ``".6g"`` format so the SAME
    geometry serialises to the SAME string on Windows and Linux/macOS — the hash
    is compared across the C++ dump echo and Python, so float formatting must
    not vary by platform, and must NOT depend on whether the caller passed ``5``
    or ``5.0`` (a dataclass field type hint does not coerce). ``.6g`` collapses
    ``5``/``5.0`` to ``"5"``; the ``+ 0.0`` normalises ``-0.0`` to ``0.0`` so the
    sign of a zero never perturbs the fingerprint.

    :param value: a finite number (a cm / degree measurement).
    :return: the ``".6g"``-formatted string of the float.
    """
    return format(float(value) + 0.0, ".6g")


def geom_hash(geom: "Any") -> str:
    """Return a stable, platform-independent fingerprint of the geometry.

    Fingerprints exactly the geometry params (:data:`_GEOM_KEYS` — grid
    dimensions, cell sizes, gaps, seat centre, heading) so the dump can echo it
    and ``player_html`` can badge a ``--trace``-against-a-re-seated-grid
    mismatch (mirrors the existing "trace recorded against a different field"
    guard).

    Platform-independence + type-independence: the canonical form is a JSON
    string with ``sort_keys=True`` and tight separators. The integer grid
    dimensions (:data:`_GEOM_INT_KEYS`) are hashed as bare ``int``; the cm /
    degree measurements (:data:`_GEOM_FLOAT_KEYS`) are hashed via
    :func:`_canonical_float` (fixed ``".6g"`` string). Because the split is by
    KEY, not by the runtime type of the value, ``cell_w_cm=5`` and
    ``cell_w_cm=5.0`` fingerprint identically — no float ``repr`` (whose output
    could differ across platforms) reaches the hash. The digest is a SHA-256
    hexdigest truncated to 16 hex chars (ample to avoid collisions for the small
    geometry space, short enough for a badge).

    :param geom: an object exposing the :data:`_GEOM_KEYS` attributes (a
        :class:`MosaicConfig` or an equivalent namespace).
    :return: a 16-char lowercase hex fingerprint string.
    """
    canonical: Dict[str, Any] = {}
    for key in _GEOM_INT_KEYS:
        canonical[key] = int(getattr(geom, key))
    for key in _GEOM_FLOAT_KEYS:
        canonical[key] = _canonical_float(getattr(geom, key))
    payload = json.dumps(canonical, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()[:16]


# ---------------------------------------------------------------------------
# Pure validators (AC3) — raise ValueError; T2 wraps as LayoutError
# ---------------------------------------------------------------------------
#
# These validators are the SINGLE source of the AC3 rules. They raise a plain
# ``ValueError`` with a precise message (the offending key + the fix). T2
# (``layout.py``) catches the ``ValueError`` and re-raises it as ``LayoutError``
# with a ``ctps.layout: mosaic.<...>`` key path — so the rule text lives here
# exactly once and T2 owns only the key-path framing. Neither validator
# constructs a ``MosaicConfig``; they operate on the raw decoded dict (the shape
# T2 holds after ``json.loads``), so a malformed dict is rejected BEFORE any
# object is built.


def validate_mosaic_config(data: Any) -> None:
    """Strictly validate a decoded ``mosaic`` block dict (AC3), raising on the first fault.

    Enforces every AC3 mosaic rule:

    * the block is a dict carrying every required key;
    * ``mode`` is ``"random"`` or ``"directed"``;
    * ``rows`` / ``cols`` are positive ints with ``rows * cols == CELL_COUNT``;
    * ``cell_w_cm`` / ``cell_h_cm`` are finite ``> 0``; ``row_gap_cm`` /
      ``col_gap_cm`` are finite ``>= 0``; ``center_*`` / ``heading_deg`` are
      finite;
    * ``palette`` is a non-empty subset of :data:`COLOR_NAMES` with no
      duplicates;
    * ``cells`` has exactly :data:`CELL_COUNT` entries, each ``None`` or a
      palette colour;
    * **directed** mode: no ``None`` cell;
    * **random** mode: ``counts`` keys are palette colours and values are
      non-negative ints; ``sum(counts.values())`` may be ANY non-negative
      number (ADR-063) — an under-fill leaves the leftover cells as air, an
      over-fill keeps a random subset (both resolved in :func:`roll_mosaic`);
    * ``seed`` is ``None`` or a non-negative ``int`` (``-1`` / ``1.5`` / a bool
      are rejected).

    Raises a plain :class:`ValueError` (NOT ``LayoutError``) so ``layout.py``
    can catch it and re-raise with a ``ctps.layout: mosaic.<...>`` key path
    without this module depending on ``layout.py`` — the rules live here once.

    :param data: the decoded ``mosaic`` block (any type — a non-dict is
        rejected).
    :raises ValueError: naming the first offending key and the fix.
    """
    if not isinstance(data, dict):
        raise ValueError(
            f"'mosaic' must be a dict; got {type(data).__name__!r}."
        )

    _validate_mode(data)
    _validate_grid_dimensions(data)
    _validate_geometry_numbers(data)
    palette = _validate_palette(data)
    null_count = _validate_cells(data, palette)
    _validate_mode_specific(data, palette, null_count)
    _validate_seed(data)


def _validate_mode(data: dict) -> None:
    """Validate the ``mode`` key is a known roll mode."""
    if "mode" not in data:
        raise ValueError("'mosaic' missing required key 'mode'.")
    mode = data["mode"]
    if mode not in _ALLOWED_MODES:
        raise ValueError(
            f"mosaic.mode must be one of {sorted(_ALLOWED_MODES)!r}; got {mode!r}."
        )


def _validate_grid_dimensions(data: dict) -> None:
    """Validate ``rows`` / ``cols`` are positive ints multiplying to CELL_COUNT."""
    for key in ("rows", "cols"):
        if key not in data:
            raise ValueError(f"'mosaic' missing required key {key!r}.")
        value = data[key]
        if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
            raise ValueError(
                f"mosaic.{key} must be a positive integer; got {value!r}."
            )
    product = data["rows"] * data["cols"]
    if product != CELL_COUNT:
        raise ValueError(
            f"mosaic.rows * mosaic.cols must equal {CELL_COUNT} "
            f"(got {data['rows']} * {data['cols']} = {product})."
        )


def _validate_geometry_numbers(data: dict) -> None:
    """Validate the finite / positivity constraints on the geometry scalars."""
    for key in ("center_x_cm", "center_y_cm", "heading_deg"):
        if key not in data:
            raise ValueError(f"'mosaic' missing required key {key!r}.")
        if not _is_finite_number(data[key]):
            raise ValueError(
                f"mosaic.{key} must be a finite number; got {data[key]!r}."
            )
    for key in ("cell_w_cm", "cell_h_cm"):
        if key not in data:
            raise ValueError(f"'mosaic' missing required key {key!r}.")
        value = data[key]
        if not _is_finite_number(value) or value <= 0:
            raise ValueError(
                f"mosaic.{key} must be a finite number > 0; got {value!r}."
            )
    for key in ("row_gap_cm", "col_gap_cm"):
        if key not in data:
            raise ValueError(f"'mosaic' missing required key {key!r}.")
        value = data[key]
        if not _is_finite_number(value) or value < 0:
            raise ValueError(
                f"mosaic.{key} must be a finite number >= 0; got {value!r}."
            )


def _validate_palette(data: dict) -> frozenset[str]:
    """Validate ``palette`` is a non-empty, duplicate-free subset of COLOR_NAMES.

    :return: the palette as a ``frozenset`` (for the cells / counts checks).
    """
    if "palette" not in data:
        raise ValueError("'mosaic' missing required key 'palette'.")
    palette = data["palette"]
    if not isinstance(palette, list) or not palette:
        raise ValueError(
            f"mosaic.palette must be a non-empty list; got {palette!r}."
        )
    seen: set[str] = set()
    for i, name in enumerate(palette):
        if name not in _COLOR_NAME_SET:
            raise ValueError(
                f"mosaic.palette[{i}] {name!r} is not a known colour; "
                f"must be one of {list(COLOR_NAMES)!r}."
            )
        if name in seen:
            raise ValueError(
                f"mosaic.palette[{i}] {name!r} is a duplicate; palette colours "
                "must be unique."
            )
        seen.add(name)
    return frozenset(seen)


def _validate_cells(data: dict, palette: frozenset[str]) -> int:
    """Validate ``cells`` is length CELL_COUNT with each entry null-or-in-palette.

    :return: the number of ``None`` (rolled) cells.
    """
    if "cells" not in data:
        raise ValueError("'mosaic' missing required key 'cells'.")
    cells = data["cells"]
    if not isinstance(cells, list):
        raise ValueError(
            f"mosaic.cells must be a list; got {type(cells).__name__!r}."
        )
    if len(cells) != CELL_COUNT:
        raise ValueError(
            f"mosaic.cells must have exactly {CELL_COUNT} entries; "
            f"got {len(cells)}."
        )
    null_count = 0
    for i, cell in enumerate(cells):
        if cell is None:
            null_count += 1
            continue
        if cell not in _COLOR_NAME_SET:
            raise ValueError(
                f"mosaic.cells[{i}] {cell!r} is not a known colour; "
                f"must be null or one of {list(COLOR_NAMES)!r}."
            )
        if cell not in palette:
            raise ValueError(
                f"mosaic.cells[{i}] pins {cell!r}, which is not in the palette "
                f"{sorted(palette)!r}."
            )
    return null_count


def _validate_mode_specific(
    data: dict, palette: frozenset[str], null_count: int
) -> None:
    """Validate the directed-no-null / random-counts colour+quota rules.

    ``null_count`` gates only the directed-mode "every cell pinned" check;
    random-mode ``counts`` may sum to ANY non-negative number (ADR-063) — an
    under-fill leaves the leftover null cells as AIR and an over-fill keeps a
    random subset, both resolved in :func:`roll_mosaic`, so no sum equality is
    enforced here.
    """
    mode = data["mode"]
    if mode == MODE_DIRECTED:
        if null_count != 0:
            raise ValueError(
                f"mosaic.mode='directed' requires every cell pinned, but "
                f"{null_count} cell(s) are null; pin them or use mode='random'."
            )
        return

    # random mode: counts keys must be palette colours and values non-negative
    # ints. sum(counts) is intentionally NOT constrained to null_count (ADR-063).
    counts = data.get("counts", {})
    if not isinstance(counts, dict):
        raise ValueError(
            f"mosaic.counts must be a dict; got {type(counts).__name__!r}."
        )
    for name, quota in counts.items():
        if name not in _COLOR_NAME_SET:
            raise ValueError(
                f"mosaic.counts key {name!r} is not a known colour; "
                f"must be one of {list(COLOR_NAMES)!r}."
            )
        if name not in palette:
            raise ValueError(
                f"mosaic.counts key {name!r} is not in the palette "
                f"{sorted(palette)!r}."
            )
        if not isinstance(quota, int) or isinstance(quota, bool) or quota < 0:
            raise ValueError(
                f"mosaic.counts[{name!r}] must be a non-negative integer; "
                f"got {quota!r}."
            )


def _validate_seed(data: dict) -> None:
    """Validate ``seed`` is absent / ``None`` or a non-negative ``int``."""
    seed = data.get("seed")
    if seed is None:
        return
    if not _is_nonneg_int(seed):
        raise ValueError(
            f"mosaic.seed must be null or a non-negative integer; got {seed!r} "
            "(a negative value or a float such as 1.5 is rejected)."
        )


def validate_vision_cone(data: Any) -> None:
    """Strictly validate a decoded ``robot.vision`` cone block (AC3), raising on the first fault.

    Enforces the AC3 cone rules:

    * the block is a dict carrying ``origin_x_cm`` / ``origin_y_cm`` /
      ``half_angle_deg`` (``range_cm`` optional);
    * ``origin_x_cm`` / ``origin_y_cm`` are finite;
    * ``half_angle_deg`` is finite and strictly inside ``(0, 90)`` — a value of
      ``0`` or ``90`` (or beyond) is rejected;
    * ``range_cm`` is ``None`` / absent (unbounded) or a finite ``> 0``.

    Raises a plain :class:`ValueError` (NOT ``LayoutError``) so ``layout.py`` can
    catch it and re-raise with a ``ctps.layout: robot.vision.<...>`` key path —
    the rule text lives here once.

    :param data: the decoded ``robot.vision`` block (any type — a non-dict is
        rejected).
    :raises ValueError: naming the first offending key and the fix.
    """
    if not isinstance(data, dict):
        raise ValueError(
            f"'robot.vision' must be a dict; got {type(data).__name__!r}."
        )

    for key in ("origin_x_cm", "origin_y_cm", "half_angle_deg"):
        if key not in data:
            raise ValueError(f"'robot.vision' missing required key {key!r}.")

    for key in ("origin_x_cm", "origin_y_cm"):
        if not _is_finite_number(data[key]):
            raise ValueError(
                f"robot.vision.{key} must be a finite number; got {data[key]!r}."
            )

    half_angle = data["half_angle_deg"]
    if not _is_finite_number(half_angle) or not (0.0 < half_angle < 90.0):
        raise ValueError(
            f"robot.vision.half_angle_deg must be a finite number strictly in "
            f"(0, 90); got {half_angle!r}."
        )

    range_cm = data.get("range_cm")
    if range_cm is not None:
        if not _is_finite_number(range_cm) or range_cm <= 0:
            raise ValueError(
                f"robot.vision.range_cm must be null (unbounded) or a finite "
                f"number > 0; got {range_cm!r}."
            )
