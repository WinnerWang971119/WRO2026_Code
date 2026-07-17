"""Sim↔firmware FSM parity support (issue #108, T5).

This module is the **Python side of the 3-place trace-schema contract** that
proves the pure-Python :class:`~tools.path_planner.sim.runtime.PathTrackerSim`
makes the same per-hop FSM decisions (and resulting pose) as the real C++
``PathTracker`` firmware. The three places that must agree byte-for-byte on
record ``kind`` strings + field names + value formats are:

1. the C++ **emitter** — ``WRO2026/PathTracker.cpp`` (the ``PT_PARITY_TRACE``
   record builders) + the T6 parity-dump driver (the ``goal_start`` /
   ``goal_done`` framing);
2. the C++-stream **loader** — :func:`load_cpp_ndjson` (here); and
3. the sim **normalizer** — :func:`normalize_sim_stream` (here).

A ``kind`` / field the emitter produces but a loader does not know about is a
**loud failure** (:class:`ParitySchemaError`), never a silent skip — mirroring
the CTPS ``drive.method`` whitelist lesson (memory
``gotcha-ctps-drive-method-contract``). Numeric fields are parsed to numbers
and **never compared as strings** (the C++ emits fixed 6-decimal text, this
side carries full floats); the actual three-bucket comparison + tolerances are
owned by T8, not here.

Public surface (the contract T7 + T8 consume)::

    PARITY_SCHEMA_VERSION                         # expected schema_version
    TraceRecord                                   # normalized record (kind + fields)
    Pose                                          # (x, y, heading) for the pose cross-check
    ParityCase                                    # one (start, goal[, offset]) sweep case
    Allowlist / AllowlistEntry                    # self-auditing divergence allowlist
    ParitySchemaError / ParityAllowlistError      # loud-failure exception types

    normalize_sim_stream(sim, case) -> list[TraceRecord]
    load_cpp_ndjson(text) -> dict[case_id, CppCase]
    integrate_pose(records, field, node_sequence, *, start_node=None,
                   start_heading_deg=None) -> list[Pose]
    load_allowlist(path) -> Allowlist

**Record representation.** A :class:`TraceRecord` carries a ``kind`` string and
a ``fields`` mapping of the kind-specific fields, with numeric fields coerced to
``float`` / ``int`` (so a comparator can map each field to a bucket by name and
both sources round-trip to the *same* representation). Framing records
(``goal_start`` / ``goal_done``) are consumed by :func:`load_cpp_ndjson` to
group + validate, and are **not** part of the per-hop record lists either side
returns (T6's driver + T8's test own that framing).

**Pose integration.** :func:`integrate_pose` runs the SAME single
:class:`~tools.path_planner.sim.chassis.VirtualChassis` over BOTH the C++-stream
records and the sim records, so there is no cross-language float drift. It
returns **one** :class:`Pose` per input record (the chassis pose after that
record's motion is applied). ``ON_INTERSECTION`` re-anchors snap the axle onto
the destination node (zero per-hop drift, memory
``gotcha-sim-chassis-axle-on-node``); ``ON_DEAD_RECKONED_ADVANCE`` hops advance
by the COMMANDED distance and accumulate the pose cross-check's genuine drift
signal (memory ``gotcha-sim-primitive-payload-is-command-not-displacement`` — a
seek's record distance is the command, not the displacement).
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from fnmatch import fnmatchcase
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Tuple

from tools.path_planner.graph_builder import (
    EDGE_BLIND,
    is_projection_node,
)
from tools.path_planner.pipeline_config import (
    BlindMode,
    LineFollowMode,
    PipelineConfig,
    SpeedTier,
    TurnMode,
)
from tools.path_planner.sim.chassis import VirtualChassis, _normalize
from tools.path_planner.sim.runtime import (
    ACT_BLIND_PASS,
    ACT_BLIND_REVERSE,
    ACT_FOLLOW_DIST,
    ACT_FOLLOW_DIST_PASS,
    ACT_FOLLOW_INT,
    ACT_FORWARD,
    ACT_HEADING_RESET,
    ACT_LINE_OVERSHOOT,
    ACT_LINE_PASS,
    ACT_LINE_REVERSE,
    ACT_PROJECTION_REVERSE,
    ACT_PROJECTION_REVERSE_PASS,
    ACT_REANCHOR,
    ACT_RETURN_FORWARD,
    ACT_RETURN_STOP_AT_LINE,
    ACT_STOP_AT_LINE,
    ACT_STOP_AT_LINE_REVERSE,
    ACT_TURN_REL,
    ACT_WALL_DEPART,
    ACT_WALL_STOP,
    LINE_BOARD_COUNT,
    LINE_SENSOR_AXLE_OFFSET_CM,
    LINE_SENSOR_AXLE_OFFSET_REAR_CM,
    PIVOT_TRIM_MIN_DRIVE_CM,
    TSHAPE_START_SPEED,
    Direction,
    EdgeDispatchQuery,
    PathTrackerSim,
    StopAtLine,
    _read_edge_stop_at_line_override,
    _speed_tier_to_pwm,
    _wall_axle_coord,
    resolve_blind_mode,
    resolve_edge_dispatch,
    resolve_line_follow_mode,
    resolve_speed,
    resolve_stop_at_line,
    walk_next_hop,
)

#: Expected ``schema_version`` on every ``goal_start`` framing record. The C++
#: dump target compiles with ``-DPARITY_SCHEMA_VERSION=3``; any mismatch on load
#: is a hard, loud failure (a desynced emitter vs loader). Bumped 2 -> 3 for the
#: issue-#156 / ADR-069 parity v3 wire schema: the two LINE-follow records gain a
#: ``follow_mode`` field and rename the historically-misnamed ``line_mode`` (which
#: has always held the resolved speed tier) to ``speed``.
PARITY_SCHEMA_VERSION: int = 3


__all__ = [
    "PARITY_SCHEMA_VERSION",
    "TRACE_RECORD_FIELDS",
    "PER_HOP_RECORD_KINDS",
    "TraceRecord",
    "Pose",
    "ParityCase",
    "CppCase",
    "Allowlist",
    "AllowlistEntry",
    "ParitySchemaError",
    "ParityAllowlistError",
    "normalize_sim_stream",
    "load_cpp_ndjson",
    "integrate_pose",
    "load_allowlist",
]


# ---------------------------------------------------------------------------
# Exceptions (loud failures — the 3-place contract is never silently skipped)
# ---------------------------------------------------------------------------


class ParitySchemaError(RuntimeError):
    """Raised on any trace-schema desync between emitter, loader and normalizer.

    An unknown record ``kind``, an unknown / missing field on a known kind, a
    wrong field type, a bad ``schema_version``, or malformed ``goal_start`` /
    ``goal_done`` framing all raise this (never a silent skip). This is the
    Python end of the 3-place contract guard (the CTPS ``drive.method``
    lesson).
    """


class ParityAllowlistError(RuntimeError):
    """Raised on a malformed allowlist file or an ambiguous allowlist match.

    A duplicate entry id, an entry whose field set does not match the schema,
    or a single mismatch that matches more than one entry (an over-broad
    allowlist) all raise this.
    """


# ---------------------------------------------------------------------------
# Normalized trace record (the cross-language wire schema, Python side)
# ---------------------------------------------------------------------------

#: Per-kind field schema (field name -> Python coercion). The ``kind`` key
#: itself is implicit; ``goal_start`` additionally carries ``schema_version``.
#: Both :func:`load_cpp_ndjson` (validating the C++ stream) and
#: :func:`normalize_sim_stream` (building the sim stream) hold to this single
#: source so the two sides round-trip to identical :class:`TraceRecord`
#: representations. Float fields are coerced (C++ emits e.g. ``12.020000`` as a
#: JSON number; this side carries full floats) so a comparator never compares
#: them as strings.
TRACE_RECORD_FIELDS: Dict[str, Dict[str, Any]] = {
    "goal_start": {
        "case_id": str,
        "start": str,
        "goal": str,
        "route_dir": str,
        "arrival_dir": str,
        "realized_offset_cm": float,
    },
    "turn": {"body_heading_deg": float, "wheel": str},
    # Parity v3 (issue #156 / ADR-069) field-set BUMP on both LINE-follow records:
    # `line_mode` is RENAMED to `speed` (its value has always been the resolved
    # SpeedTier tier name — only the key was misnamed) and a new `follow_mode`
    # (the RESOLVED LineFollowMode member name MIDDLE/LEFT/RIGHT — the MODE, not
    # the signed offset, recorded PRE-reverse-sign) is added right after it. The
    # field ORDER here mirrors the C++ `_ptEmitLineFollow` / `_ptEmitProjectionFollow`
    # serialization byte-for-byte (the strict loader keys on the field SET, but the
    # order is kept in lockstep so both sides read identically).
    "line_follow": {"reverse": bool, "speed": str, "follow_mode": str},
    "line_overshoot": {"advance_cm": float, "pass_through": bool, "reanchor": str},
    "projection_follow": {
        "reverse": bool,
        "speed": str,
        "follow_mode": str,
        "dist_cm": float,
        "pass_through": bool,
        "reanchor": str,
    },
    "stop_at_line_seek": {
        "reverse": bool,
        "drive_mode": str,
        "dist_cm": float,
        "overshoot_cm": float,
        "reanchor": str,
    },
    # ADR-059 field-set BUMP: `start_pwm` / `end_pwm` / `chain` are emitted on
    # EVERY blind_dead_reckon record (chained or not) by both the firmware
    # (`_ptEmitBlindDeadReckon`) and the sim (`normalize_sim_stream`). The strict
    # loader raises on an unknown OR missing field of a known kind, so the two
    # sides MUST land this bump together (which T7 firmware + T8 sim do). The
    # run-membership key is `chain`, deliberately NOT any sub-field named "kind"
    # (`kind` is the record-type discriminator this dict is keyed on; memory
    # `gotcha-parity-record-kind-is-discriminator`).
    "blind_dead_reckon": {
        "reverse": bool,
        "drive_mode": str,
        "dist_cm": float,
        "start_pwm": int,
        "end_pwm": int,
        "chain": bool,
        "reanchor": str,
    },
    # ADR-059: emitted at each NON-braking mid-hop node boundary of a blind-chain
    # run (the sim tag `ACT_BLIND_PASS`; the firmware `_ptEmitBlindPass`). Sits
    # BETWEEN the two adjacent hops' `blind_dead_reckon` records; the run's final
    # braked arrival emits none. Its sole sub-field is `node` — deliberately NOT
    # `kind` (the record-type discriminator).
    "blind_pass": {"node": str},
    "return_turn": {"body_heading_deg": float, "wheel": str},
    "return_move": {
        "mode": str,
        "node": str,
        "dist_cm": float,
        "heading_world_deg": float,
    },
    "wall_stop": {"node": str, "axis_idx": int, "wall_coord": float},
    "wall_reanchor": {"node": str, "axis_idx": int, "wall_coord": float},
    "wall_depart": {"node": str, "axis_idx": int, "node_coord": float},
    # ADR-052: line-arrival heading reset. Emitted ONLY on a successful bind
    # (no `bound`/`result` field — the record's mere presence means it bound).
    # `pass_kind` carries "proj" (PROJECTION pass-through) or "junction" (LINE
    # pass-through) — deliberately NOT named `kind`, which is the record-type
    # discriminator this dict is keyed on (a duplicate `"kind"` wire key would
    # be invalid JSON). See ``WRO2026/PathTracker.cpp``'s ``_ptEmitHeadingReset``.
    "heading_reset": {"node": str, "pass_kind": str, "heading_deg": float},
    "goal_done": {"final": str},
}

#: The record kinds that belong to a case's per-hop body (everything except the
#: ``goal_start`` / ``goal_done`` framing). The lists both sides compare contain
#: only these.
PER_HOP_RECORD_KINDS: frozenset = frozenset(
    k for k in TRACE_RECORD_FIELDS if k not in ("goal_start", "goal_done")
)

#: The fixed ``reanchor`` enum each re-anchoring motion record carries. Named
#: once so the sim normalizer and the C++ emitter cannot drift on the constant.
_REANCHOR_FOR_KIND: Dict[str, str] = {
    "line_overshoot": "ON_INTERSECTION",
    "projection_follow": "ON_DEAD_RECKONED_ADVANCE",
    "stop_at_line_seek": "ON_INTERSECTION",
    "blind_dead_reckon": "ON_DEAD_RECKONED_ADVANCE",
}

#: Sim ACT tags that signal a return-to-node move (used to classify whether a
#: preceding ``ACT_TURN_REL`` is a planned ``turn`` or a ``return_turn``).
_RETURN_TAGS: frozenset = frozenset({ACT_RETURN_STOP_AT_LINE, ACT_RETURN_FORWARD})


@dataclass(frozen=True)
class TraceRecord:
    """One normalized per-hop trace record (the cross-language wire schema).

    A record is a ``kind`` string plus a ``fields`` mapping of the kind-specific
    fields (see :data:`TRACE_RECORD_FIELDS`), numeric fields coerced to
    ``float`` / ``int``. Both the C++-stream loader and the sim normalizer
    produce this identical representation, so T8's comparator can iterate
    ``record.fields.items()`` and map each field to a comparison bucket by name.

    Treat as immutable (frozen): ``fields`` is not mutated after construction.

    :ivar kind: the record kind (one of :data:`TRACE_RECORD_FIELDS`).
    :ivar fields: the kind-specific fields; numeric values are ``float`` /
        ``int``, bools are ``bool``, enums / names / node names are ``str``.
    """

    kind: str
    fields: Mapping[str, Any]


@dataclass(frozen=True)
class Pose:
    """A chassis pose snapshot for the pose cross-check.

    :ivar x: world-frame x coordinate (cm).
    :ivar y: world-frame y coordinate (cm).
    :ivar heading: world-frame heading (deg, ``[0, 360)``, CCW, ``0 = +x``).
    """

    x: float
    y: float
    heading: float


@dataclass(frozen=True)
class ParityCase:
    """One enumerated parity sweep case.

    :ivar case_id: the join key ``f"{start}__{goal}__{offset_tag}"`` shared by
        both sides.
    :ivar start: the node the chassis is seated on before the ``goTo``.
    :ivar goal: the ``goTo`` destination node.
    :ivar route_dir: builder route-direction override applied to every hop.
    :ivar arrival_dir: builder arrival-direction override applied to the final
        hop only.
    :ivar offset: an optional pinned operator-offset vector ``(dx, dy, dtheta)``
        applied identically on both sides before the ``goTo`` (the source of the
        return-to-node prologue on the curated cases). ``None`` for the
        ``"clean"`` all-pairs cases. The application is the caller's (T8)
        responsibility; :func:`normalize_sim_stream` consumes whatever stream
        the already-prepared sim produces.
    """

    case_id: str
    start: str
    goal: str
    route_dir: Direction = Direction.AUTO
    arrival_dir: Direction = Direction.AUTO
    offset: Optional[Tuple[float, float, float]] = None


@dataclass(frozen=True)
class CppCase:
    """One loaded C++ parity case: its per-hop body + the realized operator offset.

    :ivar records: the case's ordered per-hop :class:`TraceRecord` body (the
        ``goal_start`` / ``goal_done`` framing stripped) — exactly what the
        :func:`load_cpp_ndjson` dict values carried before finding 1's
        realized-offset plumbing landed.
    :ivar realized_offset_cm: the realized (encoder-quantized) operator-offset
        magnitude parsed from this case's ``goal_start.realized_offset_cm``
        (``0.0`` for clean / non-offset cases). The parity test seeds the sim
        with this value so ``return_move.dist_cm`` is compared honestly instead
        of being masked by an allowlist entry (finding 1, issue #116).
    """

    records: List[TraceRecord]
    realized_offset_cm: float


# ---------------------------------------------------------------------------
# Enum-name spellings (mirror the C++ `_pt*Name` helpers byte-for-byte)
# ---------------------------------------------------------------------------


def _turn_mode_name(mode: TurnMode) -> str:
    """Spell a :class:`TurnMode` exactly as the C++ ``_ptWheelName`` does.

    ``DEFAULT`` (the inherit sentinel) maps to ``"COUNTER_ROT"`` — the firmware
    treats the zero byte as a counter-rotation; the resolver delivers a concrete
    mode before this point, but the fallback mirrors the C++ default arm.
    """
    if mode == TurnMode.PIVOT_LEFT:
        return "PIVOT_LEFT"
    if mode == TurnMode.PIVOT_RIGHT:
        return "PIVOT_RIGHT"
    return "COUNTER_ROT"


def _line_mode_name(mode: SpeedTier) -> str:
    """Spell a LINE edge's resolved :class:`SpeedTier` exactly as the C++
    ``_ptLineModeName`` does.

    The parity v3 bump (issue #156 / ADR-069) RENAMES this value's wire field from
    the historically-misnamed ``line_mode`` to ``speed``; the helper name is kept
    (like the C++ ``_ptLineModeName``) to limit churn, but it produces the
    :class:`SpeedTier` member name (``SLOW`` / ``NORMAL`` / ``FAST``), NOT a follow
    mode. ``DEFAULT`` maps to ``"NORMAL"`` (the resolver delivers a concrete tier
    before this point; the fallback mirrors the C++ default arm).
    """
    if mode == SpeedTier.SLOW:
        return "SLOW"
    if mode == SpeedTier.FAST:
        return "FAST"
    return "NORMAL"


def _follow_mode_name(mode: LineFollowMode) -> str:
    """Spell a RESOLVED :class:`LineFollowMode` exactly as the C++
    ``_ptFollowModeName`` does (issue #156 / ADR-069 parity v3).

    The value is the LineFollowMode member name (``MIDDLE`` / ``LEFT`` / ``RIGHT``)
    for the ``follow_mode`` wire key — the MODE, recorded PRE-reverse-sign, never
    the signed float offset the follower ultimately applies.
    :func:`~tools.path_planner.sim.runtime.resolve_line_follow_mode` resolves
    ``DEFAULT`` -> ``MIDDLE`` upstream, so ``DEFAULT`` is never reached here; the
    fallback to ``"MIDDLE"`` mirrors the C++ defensive default arm.
    """
    if mode == LineFollowMode.LEFT:
        return "LEFT"
    if mode == LineFollowMode.RIGHT:
        return "RIGHT"
    return "MIDDLE"


def _drive_mode_name(mode: BlindMode) -> str:
    """Spell a :class:`BlindMode` exactly as the C++ ``_ptDriveModeName`` does.

    Anything other than ``GYRO_TSHAPE`` / ``RAW_FLAT`` (``SYNC_TSHAPE``, the
    residual ``STOP_AT_LINE``, or ``DEFAULT``) maps to ``"SYNC_TSHAPE"`` — the
    resolver guarantees a concrete bundle, matching the C++ default arm.
    """
    if mode == BlindMode.GYRO_TSHAPE:
        return "GYRO_TSHAPE"
    if mode == BlindMode.RAW_FLAT:
        return "RAW_FLAT"
    return "SYNC_TSHAPE"


# ---------------------------------------------------------------------------
# C++ NDJSON loader (the 3-place contract guard, Python loader side)
# ---------------------------------------------------------------------------


def _coerce_field(kind: str, name: str, coerce: Any, value: Any) -> Any:
    """Coerce + type-validate one record field, raising loudly on a mismatch."""
    if coerce is bool:
        if not isinstance(value, bool):
            raise ParitySchemaError(
                f"record {kind!r} field {name!r}: expected bool, got "
                f"{type(value).__name__} ({value!r})"
            )
        return value
    if coerce is str:
        if not isinstance(value, str):
            raise ParitySchemaError(
                f"record {kind!r} field {name!r}: expected str, got "
                f"{type(value).__name__} ({value!r})"
            )
        return value
    if coerce is int:
        # Reject bools (a bool is an int subclass) and non-integral floats.
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ParitySchemaError(
                f"record {kind!r} field {name!r}: expected int, got "
                f"{type(value).__name__} ({value!r})"
            )
        return int(value)
    # float
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ParitySchemaError(
            f"record {kind!r} field {name!r}: expected number, got "
            f"{type(value).__name__} ({value!r})"
        )
    return float(value)


def _record_from_obj(obj: Any) -> TraceRecord:
    """Build (and strictly validate) one :class:`TraceRecord` from a JSON object.

    Raises :class:`ParitySchemaError` on an unknown ``kind``, an unknown /
    missing field, a wrong field type, or (for ``goal_start``) a bad
    ``schema_version`` — the loud-failure contract guard.
    """
    if not isinstance(obj, dict):
        raise ParitySchemaError(
            f"trace record must be a JSON object, got {type(obj).__name__}"
        )
    kind = obj.get("kind")
    if kind is None:
        raise ParitySchemaError(f"trace record missing 'kind': {obj!r}")
    if kind not in TRACE_RECORD_FIELDS:
        raise ParitySchemaError(f"unknown trace record kind {kind!r}")

    spec = TRACE_RECORD_FIELDS[kind]
    expected = set(spec) | {"kind"}
    if kind == "goal_start":
        expected.add("schema_version")
    actual = set(obj)
    if actual != expected:
        unknown = sorted(actual - expected)
        missing = sorted(expected - actual)
        raise ParitySchemaError(
            f"trace record {kind!r} field mismatch: "
            f"unknown={unknown} missing={missing}"
        )

    if kind == "goal_start":
        version = obj["schema_version"]
        if version != PARITY_SCHEMA_VERSION:
            raise ParitySchemaError(
                f"schema_version {version!r} != expected {PARITY_SCHEMA_VERSION} "
                f"(emitter / loader desync)"
            )

    fields = {
        name: _coerce_field(kind, name, coerce, obj[name])
        for name, coerce in spec.items()
    }
    return TraceRecord(kind=kind, fields=fields)


def load_cpp_ndjson(text: str) -> Dict[str, CppCase]:
    """Parse the C++ parity-dump NDJSON into per-case :class:`CppCase` values.

    Each line is one JSON object. A ``goal_start`` opens a case (keyed by its
    ``case_id`` and carrying the realized operator offset); a ``goal_done``
    closes it; the records in between are the case's per-hop body. Each returned
    :class:`CppCase` holds ONLY the per-hop records (the ``goal_start`` /
    ``goal_done`` framing is consumed for grouping + validation, matching what
    :func:`normalize_sim_stream` returns for the sim side) plus the
    ``realized_offset_cm`` read off the opening ``goal_start``.

    Every record is strictly validated (:func:`_record_from_obj`): an unknown
    ``kind``, an unknown / missing field, a wrong type, a bad ``schema_version``,
    or malformed framing raises :class:`ParitySchemaError` — never a silent
    skip (the 3-place contract guard).

    :param text: the full NDJSON dump text (one JSON object per non-blank line).
    :return: a dict mapping ``case_id`` to its :class:`CppCase` (ordered per-hop
        record list + the case's realized operator-offset magnitude).
    :raises ParitySchemaError: on any malformed line, schema desync, or framing
        error (nested cases, duplicate case_id, an orphan record, an
        unterminated case).
    """
    cases: Dict[str, CppCase] = {}
    current_id: Optional[str] = None
    current_list: Optional[List[TraceRecord]] = None

    for lineno, raw in enumerate(text.splitlines(), start=1):
        line = raw.strip()
        if not line:
            continue
        try:
            obj = json.loads(line)
        except json.JSONDecodeError as err:
            raise ParitySchemaError(f"line {lineno}: invalid JSON: {err}") from err

        record = _record_from_obj(obj)

        if record.kind == "goal_start":
            if current_id is not None:
                raise ParitySchemaError(
                    f"line {lineno}: goal_start for "
                    f"{record.fields['case_id']!r} before goal_done of "
                    f"{current_id!r} (cases must not nest)"
                )
            current_id = record.fields["case_id"]
            if current_id in cases:
                raise ParitySchemaError(
                    f"line {lineno}: duplicate case_id {current_id!r}"
                )
            # Wrap the (still-empty) per-hop list in its CppCase now, stamping the
            # realized offset off this goal_start. The list is appended to below
            # (the CppCase holds the SAME list object), so the body fills in place.
            current_list = []
            cases[current_id] = CppCase(
                records=current_list,
                realized_offset_cm=record.fields["realized_offset_cm"],
            )
        elif record.kind == "goal_done":
            if current_id is None:
                raise ParitySchemaError(
                    f"line {lineno}: goal_done with no open case"
                )
            current_id = None
            current_list = None
        else:
            if current_list is None:
                raise ParitySchemaError(
                    f"line {lineno}: record {record.kind!r} outside any case "
                    f"(missing goal_start)"
                )
            current_list.append(record)

    if current_id is not None:
        raise ParitySchemaError(
            f"unterminated case {current_id!r} (missing goal_done)"
        )
    return cases


# ---------------------------------------------------------------------------
# Sim stream normalizer (maps the sim's real ACT_* stream -> TraceRecords)
# ---------------------------------------------------------------------------


@dataclass
class _HopResolution:
    """Per-hop enrichment the sim's ACT tuples do not carry, resolved in lockstep.

    Built by walking ``NEXT_HOP`` from the case's start to its goal (the SAME
    walk + pivot-latch the FSM's ``_walk_hops`` performs), so the wheel /
    drive-mode / overshoot the bare ACT tuples omit come from the sim's OWN
    resolution, never a fresh independent re-derivation.

    :ivar u: arrive-from node of the hop.
    :ivar v: destination node of the hop.
    :ivar edge: the directed graph edge attribute mapping.
    :ivar departing: the pivot being LEFT on this hop's pre-edge turn
        (``find_pivot_turn(prev, u, v)``), or ``None`` — supplies the turn
        wheel (mirror of the C++ ``_departingPivot.wheel``).
    :ivar approaching: the pivot being ENTERED on this hop
        (``find_pivot_turn(u, v, next_next)``), or ``None`` — supplies the seek
        overshoot trim.
    """

    u: str
    v: str
    edge: Mapping[str, Any]
    departing: Any
    approaching: Any

    def wheel(self, cfg: PipelineConfig) -> TurnMode:
        """Resolve the pre-edge turn wheel (departing pivot, else pipeline turn).

        Mirror of the C++ ``resolvedTurnMode = _departingPivot.active ?
        _departingPivot.wheel : PIPELINE_CONFIG.turn``.
        """
        if self.departing is not None:
            return self.departing.wheel
        return cfg.turn

    def blind_decision(self, reverse: bool, cfg: PipelineConfig, graph, field):
        """Run the sim's own classifier for this BLIND hop (drive mode + overshoot).

        Constructs the SAME :class:`EdgeDispatchQuery` ``_walk_hops`` builds (the
        direction comes from the sim's already-baked ``reverse`` tag, the pivot
        trims from this hop's latch, the seek decision from
        :func:`resolve_stop_at_line`) and returns the
        :class:`~tools.path_planner.sim.runtime.EdgeDispatchDecision` — so the
        surfaced ``drive_mode`` / ``overshoot_cm`` are the sim's real resolution,
        not a re-derivation.
        """
        direction = Direction.REAR if reverse else Direction.FRONT
        dest_is_projection = is_projection_node(graph, self.v)
        auto_rule_active = (
            bool(getattr(field, "BLIND_PROJECTION_AUTO_SEEK", True))
            and dest_is_projection
        )
        stop_at_line = resolve_stop_at_line(
            StopAtLine.AUTO,
            _read_edge_stop_at_line_override(self.edge),
            resolve_blind_mode(self.edge, cfg),
            cfg.blind,
            direction,
            LINE_BOARD_COUNT,
            auto_rule_active,
        )
        axle_offset_cm = (
            LINE_SENSOR_AXLE_OFFSET_REAR_CM
            if reverse
            else LINE_SENSOR_AXLE_OFFSET_CM
        )
        query = EdgeDispatchQuery(
            edge_type=EDGE_BLIND,
            dest_is_projection=dest_is_projection,
            direction=direction,
            raw_distance_cm=float(self.edge["distance_cm"]),
            line_mode=cfg.speed,
            stop_at_line=stop_at_line,
            proj_pass_through=False,
            departing_trim_cm=(
                self.departing.trim_cm if self.departing is not None else 0.0
            ),
            approaching_trim_cm=(
                self.approaching.trim_cm if self.approaching is not None else 0.0
            ),
            axle_offset_cm=axle_offset_cm,
            min_drive_cm=PIVOT_TRIM_MIN_DRIVE_CM,
        )
        return resolve_edge_dispatch(query)


def _resolve_hops(
    sim: PathTrackerSim,
    graph,
    goal: str,
    path: List[str],
) -> List[_HopResolution]:
    """Resolve each planned hop's pivot latch by walking ``NEXT_HOP`` in lockstep.

    The departing / approaching pivots are derived from the node triples exactly
    as the FSM's ``_walk_hops`` latch threads them (snapshot-before-overwrite):
    for hop ``i`` the departing pivot is ``find_pivot_turn(path[i-1], path[i],
    path[i+1])`` (``None`` for the first hop) and the approaching pivot is
    ``find_pivot_turn(path[i], path[i+1], path[i+2])`` (``None`` on the last
    hop). The pivot table is a pure node-triple lookup, so this stateless form
    is byte-identical to the FSM's single-carry latch.
    """
    hops: List[_HopResolution] = []
    n = len(path)
    for i in range(n - 1):
        u = path[i]
        v = path[i + 1]
        edge = graph.edges[u, v]
        is_last = v == goal
        departing = sim.find_pivot_turn(path[i - 1], u, v) if i >= 1 else None
        approaching = None
        if not is_last and (i + 2) < n:
            approaching = sim.find_pivot_turn(u, v, path[i + 2])
        hops.append(
            _HopResolution(
                u=u, v=v, edge=edge, departing=departing, approaching=approaching
            )
        )
    return hops


def _make_turn(kind: str, body_heading_deg: float, wheel: TurnMode) -> TraceRecord:
    return TraceRecord(
        kind,
        {"body_heading_deg": float(body_heading_deg), "wheel": _turn_mode_name(wheel)},
    )


def _make_return_move(
    mode: str, node: str, dist_cm: float, heading_world_deg: float
) -> TraceRecord:
    return TraceRecord(
        "return_move",
        {
            "mode": mode,
            "node": str(node),
            "dist_cm": float(dist_cm),
            "heading_world_deg": float(heading_world_deg),
        },
    )


def _make_line_follow(reverse: bool, speed: str, follow_mode: str) -> TraceRecord:
    return TraceRecord(
        "line_follow",
        {"reverse": bool(reverse), "speed": speed, "follow_mode": follow_mode},
    )


def _make_line_overshoot(advance_cm: float, pass_through: bool) -> TraceRecord:
    return TraceRecord(
        "line_overshoot",
        {
            "advance_cm": float(advance_cm),
            "pass_through": bool(pass_through),
            "reanchor": _REANCHOR_FOR_KIND["line_overshoot"],
        },
    )


def _make_projection_follow(
    reverse: bool, speed: str, follow_mode: str, dist_cm: float, pass_through: bool
) -> TraceRecord:
    return TraceRecord(
        "projection_follow",
        {
            "reverse": bool(reverse),
            "speed": speed,
            "follow_mode": follow_mode,
            "dist_cm": float(dist_cm),
            "pass_through": bool(pass_through),
            "reanchor": _REANCHOR_FOR_KIND["projection_follow"],
        },
    )


def _make_stop_at_line_seek(
    reverse: bool, drive_mode: str, dist_cm: float, overshoot_cm: float
) -> TraceRecord:
    return TraceRecord(
        "stop_at_line_seek",
        {
            "reverse": bool(reverse),
            "drive_mode": drive_mode,
            "dist_cm": float(dist_cm),
            "overshoot_cm": float(overshoot_cm),
            "reanchor": _REANCHOR_FOR_KIND["stop_at_line_seek"],
        },
    )


def _make_blind_dead_reckon(
    reverse: bool,
    drive_mode: str,
    dist_cm: float,
    start_pwm: int,
    end_pwm: int,
    chain: bool,
) -> TraceRecord:
    """Build a ``blind_dead_reckon`` record (ADR-059 field-set bump).

    ``start_pwm`` / ``end_pwm`` are the RESOLVED velocity-plan endpoints (signed
    int16), ``chain`` whether this hop is part of an ADR-059 chained run — the
    three fields the firmware ``_ptEmitBlindDeadReckon`` emits on EVERY record of
    this kind. The strict schema requires them, so both sides carry them always.
    """
    return TraceRecord(
        "blind_dead_reckon",
        {
            "reverse": bool(reverse),
            "drive_mode": drive_mode,
            "dist_cm": float(dist_cm),
            "start_pwm": int(start_pwm),
            "end_pwm": int(end_pwm),
            "chain": bool(chain),
            "reanchor": _REANCHOR_FOR_KIND["blind_dead_reckon"],
        },
    )


def _make_blind_pass(node: str) -> TraceRecord:
    """Build a ``blind_pass`` record (ADR-059). The sole field is ``node``.

    Emitted at each NON-braking mid-hop node boundary of a blind-chain run (the
    sim's :data:`~tools.path_planner.sim.runtime.ACT_BLIND_PASS` tag; the
    firmware's `_ptEmitBlindPass`). It carries no motion (the chained hop already
    advanced onto ``node``), so :func:`integrate_pose` treats it as a NO-OP.
    """
    return TraceRecord("blind_pass", {"node": str(node)})


def _make_wall(
    kind: str, node: str, axis_idx: int, coord_key: str, coord: float
) -> TraceRecord:
    return TraceRecord(
        kind, {"node": str(node), "axis_idx": int(axis_idx), coord_key: float(coord)}
    )


def _make_heading_reset(node: str, pass_kind: str, heading_deg: float) -> TraceRecord:
    """Build a ``heading_reset`` record (ADR-052). Model on :func:`_make_wall`.

    Emitted only for a successful bind (:func:`normalize_sim_stream` only calls
    this from the :data:`~tools.path_planner.sim.runtime.ACT_HEADING_RESET` arm,
    which the sim yields only on the accept path).
    """
    return TraceRecord(
        "heading_reset",
        {"node": str(node), "pass_kind": str(pass_kind), "heading_deg": float(heading_deg)},
    )


def normalize_sim_stream(sim: PathTrackerSim, case: ParityCase) -> List[TraceRecord]:
    """Normalize the sim's real per-hop stream for one case into TraceRecords.

    Drives off the sim's REAL emitted stream (``sim.step_through``) so the
    cross-hop SEQUENCING — the pass-through latch, the pivot carry, the
    return-to-node prologue and the wall-reset interlude — comes from the actual
    ``_walk_hops`` FSM, never a re-implementation. Each FRONT/REAR ACT-tuple pair
    is collapsed to one ``{kind, reverse}`` record matching the C++
    ``{primitive, reverse}`` emission, and the fields the bare ACT tuples do not
    carry are ENRICHED from the sim's own resolution (NOT from the C++ stream):

    * ``body_heading_deg`` (turn / return_turn) — the post-turn chassis heading
      from the step snapshot (the sim's resolved body heading).
    * ``wheel`` (turn / return_turn) — the departing pivot's wheel for a planned
      turn, else ``PIPELINE_CONFIG.turn``; ``PIPELINE_CONFIG.turn`` for a return
      turn (the firmware routes returns through the pipeline turn). Surfaced
      from the sim's pivot / pipeline resolution, never recomputed in a shared
      normalizer (which would be vacuous agreement).
    * ``speed`` (line_follow / projection_follow) — ``resolve_speed(edge, cfg)``
      for that hop's edge (parity v3 renamed this wire field from the
      historically-misnamed ``line_mode``; the value is still the ``speed`` tier).
    * ``follow_mode`` (line_follow / projection_follow) —
      ``resolve_line_follow_mode(edge, cfg)`` for that hop's edge, stringified to
      the RESOLVED :class:`~tools.path_planner.pipeline_config.LineFollowMode`
      member name (``MIDDLE`` / ``LEFT`` / ``RIGHT``, never ``DEFAULT``) — the MODE,
      recorded PRE-reverse-sign (issue #156 / ADR-069). The reverse-board sign the
      firmware composes downstream is deliberately NOT applied here (this is the
      mode, not the signed offset).
    * ``drive_mode`` + ``overshoot_cm`` (seek / dead-reckon) — the sim's own
      :func:`resolve_edge_dispatch` decision for that hop.

    The bare ACT tuples carry the rest directly: ``reverse`` (the FRONT vs REAR
    tag), ``pass_through`` (the ``_PASS`` tag), the commanded distances /
    advances, the re-anchor kind, and the wall node / axis / coordinates.

    Does NOT emit ``goal_start`` / ``goal_done`` — that framing belongs to the
    T6 driver and the T8 test.

    The caller (T8) constructs + seats the sim and applies the case's pinned
    operator offset (for offset-tagged curated cases) BEFORE calling this; the
    normalizer consumes whatever stream the prepared sim produces.

    :param sim: a seated, zeroed :class:`PathTrackerSim` (its
        ``localization.current_node()`` is the case start; any pinned operator
        offset already applied). This call runs the sim to the goal (mutating
        it), so the sim is single-use per case.
    :param case: the :class:`ParityCase`; ``goal`` / ``route_dir`` /
        ``arrival_dir`` drive the ``step_through``.
    :return: the ordered per-hop :class:`TraceRecord` list for the case.
    :raises ParitySchemaError: if the sim emits an ACT tag this normalizer does
        not know (a sim-side 3-place contract desync).
    """
    field = sim._field_data    # noqa: SLF001 — read-only sim accessors (cf. demo.py)
    graph = sim._graph         # noqa: SLF001
    next_hop = sim._next_hop   # noqa: SLF001
    cfg = PipelineConfig.from_field_data(field)

    start = sim.localization.current_node()
    goal = case.goal

    # Lockstep NEXT_HOP walk for per-hop enrichment — built BEFORE step_through
    # mutates the sim (walk_next_hop + find_pivot_turn are read-only).
    path = walk_next_hop(next_hop, start, goal)
    hops = _resolve_hops(sim, graph, goal, path)

    # The sim's REAL emitted stream (sequencing source of truth).
    stream = list(
        sim.step_through(
            goal,
            route_direction=case.route_dir,
            arrival_direction=case.arrival_dir,
        )
    )

    records: List[TraceRecord] = []
    hop_idx = 0
    n = len(stream)
    i = 0
    while i < n:
        prim, snap = stream[i]
        tag = prim[0]

        if tag == ACT_TURN_REL:
            # Lookahead: a pre-edge turn is a return_turn iff the NEXT primitive
            # is a return move. stream[i + 1] is a (primitive, snapshot) pair, so
            # the next tag is stream[i + 1][0][0].
            is_return = (i + 1 < n) and (stream[i + 1][0][0] in _RETURN_TAGS)
            if is_return:
                records.append(
                    _make_turn("return_turn", snap.heading_deg, cfg.turn)
                )
            else:
                records.append(
                    _make_turn("turn", snap.heading_deg, hops[hop_idx].wheel(cfg))
                )

        elif tag == ACT_RETURN_STOP_AT_LINE:
            # (tag, node, dist_cm, heading_world_deg)
            records.append(
                _make_return_move("seek", prim[1], prim[2], snap.heading_deg)
            )
        elif tag == ACT_RETURN_FORWARD:
            # (tag, node, dist_cm) — the world heading is the faced return heading.
            records.append(
                _make_return_move("forward", prim[1], prim[2], snap.heading_deg)
            )

        elif tag in (ACT_FOLLOW_INT, ACT_LINE_REVERSE):
            # line_follow start of a LINE->junction hop (overshoot completes it).
            reverse = tag == ACT_LINE_REVERSE
            edge = hops[hop_idx].edge
            speed = _line_mode_name(resolve_speed(edge, cfg))
            follow_mode = _follow_mode_name(resolve_line_follow_mode(edge, cfg))
            records.append(_make_line_follow(reverse, speed, follow_mode))
        elif tag in (ACT_LINE_OVERSHOOT, ACT_LINE_PASS):
            # (tag, advance_cm) — completes the LINE->junction hop.
            records.append(_make_line_overshoot(prim[1], tag == ACT_LINE_PASS))
            hop_idx += 1

        elif tag in (
            ACT_FOLLOW_DIST,
            ACT_FOLLOW_DIST_PASS,
            ACT_PROJECTION_REVERSE,
            ACT_PROJECTION_REVERSE_PASS,
        ):
            reverse = tag in (ACT_PROJECTION_REVERSE, ACT_PROJECTION_REVERSE_PASS)
            pass_through = tag in (ACT_FOLLOW_DIST_PASS, ACT_PROJECTION_REVERSE_PASS)
            dist_cm = prim[2] if reverse else prim[1]
            edge = hops[hop_idx].edge
            speed = _line_mode_name(resolve_speed(edge, cfg))
            follow_mode = _follow_mode_name(resolve_line_follow_mode(edge, cfg))
            records.append(
                _make_projection_follow(
                    reverse, speed, follow_mode, dist_cm, pass_through
                )
            )
            hop_idx += 1

        elif tag in (ACT_STOP_AT_LINE, ACT_STOP_AT_LINE_REVERSE):
            reverse = tag == ACT_STOP_AT_LINE_REVERSE
            dist_cm = prim[2] if reverse else prim[1]
            decision = hops[hop_idx].blind_decision(reverse, cfg, graph, field)
            records.append(
                _make_stop_at_line_seek(
                    reverse,
                    _drive_mode_name(decision.drive_mode),
                    dist_cm,
                    decision.overshoot_cm,
                )
            )
            hop_idx += 1

        elif tag in (ACT_FORWARD, ACT_BLIND_REVERSE):
            reverse = tag == ACT_BLIND_REVERSE
            dist_cm = prim[2] if reverse else prim[1]
            decision = hops[hop_idx].blind_decision(reverse, cfg, graph, field)
            drive_mode = _drive_mode_name(decision.drive_mode)

            # ADR-059 velocity-plan endpoints. The sim (runtime._walk_hops)
            # appends a `(start_pwm, end_pwm, chain)` suffix to the ACT tuple
            # whenever it resolved a NON-default plan — a chained hop OR a
            # pass-through-seeded lone hop. When present, use it verbatim; a
            # PRISTINE tuple (no suffix) is a cold-start non-chain hop, whose
            # endpoints are computed here to match the firmware:
            #   RAW_FLAT : start = end = +/-cruise (its flat open-loop plan), where
            #              cruise is this edge's resolved SpeedTier PWM (ADR-064:
            #              a `speed: FAST` RAW_FLAT hop runs flat at 90, not 70)
            #   tShape   : start = +/-TSHAPE_START_SPEED, end = 0 (braked stop)
            # (memory `gotcha-ctps-mock-noop-setter-drops-builder-knob`: the plan
            # endpoints are a load-bearing knob — never dropped to a default that
            # differs from the firmware's emitted value.)
            suffix_at = 4 if reverse else 2
            if len(prim) > suffix_at + 2:
                start_pwm = int(prim[suffix_at])
                end_pwm = int(prim[suffix_at + 1])
                chain = bool(prim[suffix_at + 2])
            else:
                chain = False
                if decision.drive_mode == BlindMode.RAW_FLAT:
                    cruise_mag = _speed_tier_to_pwm(
                        resolve_speed(hops[hop_idx].edge, cfg)
                    )
                    cruise = -cruise_mag if reverse else cruise_mag
                    start_pwm = cruise
                    end_pwm = cruise
                else:
                    start_pwm = -TSHAPE_START_SPEED if reverse else TSHAPE_START_SPEED
                    end_pwm = 0

            records.append(
                _make_blind_dead_reckon(
                    reverse, drive_mode, dist_cm, start_pwm, end_pwm, chain
                )
            )
            hop_idx += 1

        elif tag == ACT_BLIND_PASS:
            # (tag, node) — ADR-059 blind-chain NON-braking mid-hop boundary
            # marker. It carries NO motion of its own (the chained
            # ACT_FORWARD/ACT_BLIND_REVERSE hop already advanced onto `node` and
            # advanced `hop_idx`), so this records `blind_pass` WITHOUT touching
            # `hop_idx`. Sits between the two adjacent hops' `blind_dead_reckon`
            # records, mirroring the firmware `_ptEmitBlindPass` stream order.
            records.append(_make_blind_pass(prim[1]))

        elif tag == ACT_WALL_STOP:
            # (tag, node, axis_idx, wall_coord)
            records.append(
                _make_wall("wall_stop", prim[1], prim[2], "wall_coord", prim[3])
            )
        elif tag == ACT_REANCHOR:
            # (tag, node, axis_idx, wall_coord)
            records.append(
                _make_wall("wall_reanchor", prim[1], prim[2], "wall_coord", prim[3])
            )
        elif tag == ACT_WALL_DEPART:
            # (tag, node, axis_idx, node_coord)
            records.append(
                _make_wall("wall_depart", prim[1], prim[2], "node_coord", prim[3])
            )

        elif tag == ACT_HEADING_RESET:
            # (tag, node, pass_kind, heading_deg) — ADR-052 line-arrival heading
            # reset, emitted only on a successful bind (the sim never
            # strict-rejects). Does NOT advance `hop_idx`: it belongs to the hop
            # whose own primitive (line_overshoot / projection_follow) already
            # advanced it a moment earlier in this same stream.
            records.append(_make_heading_reset(prim[1], prim[2], prim[3]))

        else:
            raise ParitySchemaError(
                f"normalize_sim_stream: unknown sim ACT tag {tag!r} "
                f"(sim-side 3-place schema desync)"
            )

        i += 1

    return records


# ---------------------------------------------------------------------------
# Pose integration (ONE shared VirtualChassis feeds BOTH record streams)
# ---------------------------------------------------------------------------


def _set_axis(chassis: VirtualChassis, axis_idx: int, coord: float) -> None:
    """Set the chassis on-axis coordinate (``axis_idx`` 0 = x, 1 = y)."""
    if axis_idx == 0:
        chassis.x_cm = float(coord)
    else:
        chassis.y_cm = float(coord)


def _snap_to_node(chassis: VirtualChassis, field, node: str) -> None:
    """Snap the chassis (x, y) onto a mapped node coordinate (re-anchor)."""
    x, y = field.NODES[node]
    chassis.x_cm = float(x)
    chassis.y_cm = float(y)


def _apply_wall(chassis: VirtualChassis, field, record: TraceRecord) -> None:
    """Reproduce one wall-reset interlude phase exactly as ``_dispatch_wall_reset``.

    ``facing`` is not on the wire (neither the sim ACT tuple nor the C++ record
    carries it), so it is read from the reset node's ``RESET_NODES`` spec — both
    sides run the SAME ``field`` through this function, so the facing is shared.
    """
    kind = record.kind
    node = record.fields["node"]
    axis_idx = record.fields["axis_idx"]
    info = field.RESET_NODES[node]
    # Quantize identically to `_dispatch_wall_reset` (runtime.py), which itself
    # mirrors the firmware's emitted `ResetNode.facing` int16 field
    # (header_writer._resolve_reset_rows, header_writer.py:884): round to the
    # nearest integer degree, then reduce mod 360 so a value rounding UP to 360
    # (e.g. 359.7 -> 360) wraps to 0. A plain float `_normalize` here would
    # leave a fractional facing un-rounded and diverge from the firmware's
    # quantized heading — the mod-360 already yields the `[0, 360)` contract,
    # so no further `_normalize` call is needed.
    facing = float(int(round(float(info["facing"]))) % 360)

    if kind == "wall_stop":
        # Phase 1: square to the wall normal, press the on-axis coord to the wall.
        chassis.turn_to_heading(facing)
        _set_axis(chassis, axis_idx, record.fields["wall_coord"])
    elif kind == "wall_reanchor":
        # Phase 2: re-anchor the on-axis belief to the wheel axle (node side).
        wall_coord = record.fields["wall_coord"]
        node_coord = float(field.NODES[node][axis_idx])
        axle_coord = _wall_axle_coord(wall_coord, node_coord)
        _set_axis(chassis, axis_idx, axle_coord)
    else:    # wall_depart
        # Phase 3: depart forward off the wall, on-axis belief back to the node.
        _set_axis(chassis, axis_idx, record.fields["node_coord"])


def integrate_pose(
    records: List[TraceRecord],
    field,
    node_sequence: List[str],
    *,
    start_node: Optional[str] = None,
    start_heading_deg: Optional[float] = None,
) -> List[Pose]:
    """Integrate a record stream through ONE shared chassis into per-record poses.

    The SAME function runs on both the C++-stream records and the sim records,
    so there is no cross-language float drift in the pose cross-check. Returns
    **one** :class:`Pose` per input record (the chassis pose after that record's
    motion is applied), so the two sides align record-for-record.

    Per-record motion model (mirrors the sim's chassis + re-anchor semantics):

    * ``turn`` / ``return_turn`` — ``turn_to_heading(body_heading_deg)``.
    * ``line_follow`` — no position change (the follow-phase advance is moot
      once the ``line_overshoot`` re-anchor snaps onto the node).
    * ``line_overshoot`` (``ON_INTERSECTION``) — the LINE->junction hop nets the
      edge length so the axle lands ON the node; SNAP to the destination node
      coordinate (zero per-hop drift).
    * ``stop_at_line_seek`` (``ON_INTERSECTION``) — re-anchors on the crossing,
      so SNAP to the destination node coordinate. (The seek's physical
      displacement is the trimmed advance, not the commanded ``dist_cm``, but
      the snap supersedes it.)
    * ``projection_follow`` / ``blind_dead_reckon`` (``ON_DEAD_RECKONED_ADVANCE``)
      — advance by the COMMANDED ``dist_cm`` (forward if not ``reverse``, else
      reverse) and do NOT snap; this is the pose cross-check's genuine drift
      signal.
    * ``return_move`` — the return lands back on the anchored node; SNAP to the
      record's ``node``.
    * ``wall_stop`` / ``wall_reanchor`` / ``wall_depart`` — reproduce the reset
      interlude (square to facing, press to the wall, re-anchor the axle, depart
      to the node), ending on the node.

    No per-junction offset is re-introduced (memory
    ``gotcha-sim-chassis-axle-on-node``): a clean run nets exactly the edge
    length per hop, drift ~ 0.

    :param records: the per-hop :class:`TraceRecord` list for one case (the
        output of :func:`normalize_sim_stream` or one value of
        :func:`load_cpp_ndjson`). Must NOT include ``goal_start`` / ``goal_done``.
    :param field: the field-data namespace (``NODES``, ``RESET_NODES``,
        ``START_NODE``, ``START_HEADING_DEG``). BOTH sides must pass the SAME
        ``field`` so the shared chassis + reset facings match.
    :param node_sequence: the per-hop DESTINATION-node names, in hop order —
        ``walk_next_hop(next_hop, start, goal)[1:]``. Consumed by the
        ``ON_INTERSECTION`` records (which do not carry a node name) via a hop
        cursor that advances once per planned hop. T8 provides it from the
        fixture's ``NEXT_HOP`` walk.
    :param start_node: optional node to seat the chassis on (defaults to
        ``field.START_NODE``). Pass the case's start for physical fidelity; the
        two-sided comparison holds regardless because both sides seat identically.
    :param start_heading_deg: optional start heading (defaults to
        ``field.START_HEADING_DEG``).
    :return: one :class:`Pose` per record, in order.
    :raises ParitySchemaError: on an unhandled record kind, or if the hop cursor
        runs past ``node_sequence`` (a record / node-sequence misalignment).
    """
    chassis = VirtualChassis.from_field_data(field)
    if start_node is not None:
        x, y = field.NODES[start_node]
        chassis.x_cm = float(x)
        chassis.y_cm = float(y)
    if start_heading_deg is not None:
        chassis.heading_deg = _normalize(float(start_heading_deg))

    poses: List[Pose] = []
    hop_cursor = 0

    def _dest_node() -> str:
        if hop_cursor >= len(node_sequence):
            raise ParitySchemaError(
                "integrate_pose: hop cursor "
                f"{hop_cursor} past node_sequence (len {len(node_sequence)}) "
                "— record stream / node_sequence misalignment"
            )
        return node_sequence[hop_cursor]

    for record in records:
        kind = record.kind
        fields = record.fields

        if kind in ("turn", "return_turn"):
            chassis.turn_to_heading(fields["body_heading_deg"])
        elif kind == "line_follow":
            pass    # follow-phase advance is moot; the overshoot snaps
        elif kind == "line_overshoot":
            _snap_to_node(chassis, field, _dest_node())
            hop_cursor += 1
        elif kind == "stop_at_line_seek":
            _snap_to_node(chassis, field, _dest_node())
            hop_cursor += 1
        elif kind in ("projection_follow", "blind_dead_reckon"):
            if fields["reverse"]:
                chassis.reverse(fields["dist_cm"])
            else:
                chassis.forward(fields["dist_cm"])
            hop_cursor += 1
        elif kind == "return_move":
            _snap_to_node(chassis, field, fields["node"])
        elif kind in ("wall_stop", "wall_reanchor", "wall_depart"):
            _apply_wall(chassis, field, record)
        elif kind == "heading_reset":
            # ADR-052: heading-only re-anchor, NO-OP on pose. Unlike
            # `wall_reanchor` (which corrects a real on-axis coordinate against
            # a physical wall), the sim's heading is already ground truth — there
            # is no drift to correct, so applying a heading reset changes
            # nothing in (x, y, heading). Do NOT route through `_apply_wall`
            # (it reads `field.RESET_NODES[node]`, which KeyErrors on a
            # non-reset pass-through node).
            pass
        elif kind == "blind_pass":
            # ADR-059: a blind-chain NON-braking boundary marker. The chained
            # `blind_dead_reckon` hop that precedes it already advanced onto (and
            # re-anchored at) this node, so the pass marker is a pose NO-OP — like
            # `heading_reset`, it changes nothing in (x, y, heading) and does NOT
            # advance the hop cursor (it is a boundary, not a hop).
            pass
        else:
            raise ParitySchemaError(
                f"integrate_pose: unhandled record kind {kind!r}"
            )

        poses.append(Pose(chassis.x_cm, chassis.y_cm, chassis.heading_deg))

    return poses


# ---------------------------------------------------------------------------
# Self-auditing divergence allowlist
# ---------------------------------------------------------------------------

#: The exact field set each allowlist entry must carry (no more, no less).
_ALLOWLIST_ENTRY_FIELDS: frozenset = frozenset(
    {
        "id",
        "case_selector",
        "event_kind",
        "field_path",
        "side",
        "reason",
        "tracking_issue",
    }
)


@dataclass(frozen=True)
class AllowlistEntry:
    """One intentional-divergence allowlist entry.

    :ivar id: a unique entry id (the staleness report keys on it).
    :ivar case_selector: a glob over ``case_id`` (``fnmatchcase`` — case-sensitive
        on every OS); scope an entry, never ``"*"`` combined with a wildcard kind
        + field to mute everything.
    :ivar event_kind: the record ``kind`` the divergence is on.
    :ivar field_path: the field name within that record.
    :ivar side: descriptive-only metadata noting which side is expected to differ
        (``"python"`` / ``"cpp"``). NOT consulted by :meth:`Allowlist.match` (nor
        at load time): the parity diff is symmetric — ``cpp_value`` and
        ``sim_value`` are peers with no oracle for which side is "wrong" — so
        ``side`` cannot be a match key.
    :ivar reason: a human-readable justification.
    :ivar tracking_issue: the tracking-issue reference.
    """

    id: str
    case_selector: str
    event_kind: str
    field_path: str
    side: str
    reason: str
    tracking_issue: str


class Allowlist:
    """A queryable, self-auditing set of intentional-divergence entries.

    T8 calls :meth:`match` for every observed mismatch; an unmatched mismatch is
    a real failure, a mismatch matching more than one entry is an ambiguous
    (over-broad) allowlist, and an entry that matches NOTHING across a whole run
    is stale (:meth:`stale_entries`) — which T8 fails on. This module only loads
    + exposes the entries + the matcher; the comparison / bucketing that decides
    what counts as a mismatch is T8's.
    """

    def __init__(self, entries: List[AllowlistEntry]) -> None:
        self._entries: List[AllowlistEntry] = list(entries)
        self._hits: Dict[str, int] = {}
        for entry in self._entries:
            if entry.id in self._hits:
                raise ParityAllowlistError(
                    f"duplicate allowlist entry id {entry.id!r}"
                )
            self._hits[entry.id] = 0

    @property
    def entries(self) -> List[AllowlistEntry]:
        """Return a copy of the loaded entries."""
        return list(self._entries)

    def match(
        self, case_id: str, event_kind: str, field_path: str
    ) -> Optional[AllowlistEntry]:
        """Return the single entry covering a mismatch, marking it used.

        :param case_id: the mismatch's case id (matched against
            ``case_selector`` via ``fnmatchcase`` — case-sensitive on every OS).
        :param event_kind: the mismatch's record kind (exact match).
        :param field_path: the mismatch's field name (exact match).
        :return: the matching :class:`AllowlistEntry`, or ``None`` if none
            matches (a real, non-allowlisted divergence).
        :raises ParityAllowlistError: if more than one entry matches (an
            ambiguous, over-broad allowlist).
        """
        # `side` is descriptive-only metadata and is intentionally NOT a filter
        # here: the parity diff is symmetric — `cpp_value` and `sim_value` are
        # peers with no oracle for which side is "wrong" — so `side` cannot be a
        # match key. `fnmatchcase` (not `fnmatch`) keeps globbing case-sensitive
        # on every OS (`fnmatch` normcases the path, so it is case-insensitive on
        # Windows).
        matched = [
            entry
            for entry in self._entries
            if entry.event_kind == event_kind
            and entry.field_path == field_path
            and fnmatchcase(case_id, entry.case_selector)
        ]
        if len(matched) > 1:
            raise ParityAllowlistError(
                f"mismatch ({case_id!r}, {event_kind!r}, {field_path!r}) "
                f"matches {len(matched)} allowlist entries: "
                f"{[e.id for e in matched]}"
            )
        if not matched:
            return None
        self._hits[matched[0].id] += 1
        return matched[0]

    def stale_entries(self) -> List[AllowlistEntry]:
        """Return entries that have matched zero mismatches (stale -> fail CI)."""
        return [entry for entry in self._entries if self._hits[entry.id] == 0]


def load_allowlist(path) -> Allowlist:
    """Load the JSON divergence allowlist into a queryable :class:`Allowlist`.

    :param path: filesystem path to the allowlist JSON (a list of entry
        objects, each with exactly the :class:`AllowlistEntry` field set).
    :return: the loaded :class:`Allowlist`.
    :raises ParityAllowlistError: if the file is not a JSON list, or any entry's
        field set does not match the schema, or an entry id is duplicated.
    """
    raw_text = Path(path).read_text(encoding="utf-8")
    data = json.loads(raw_text)
    if not isinstance(data, list):
        raise ParityAllowlistError(
            f"allowlist must be a JSON list, got {type(data).__name__}"
        )
    entries: List[AllowlistEntry] = []
    for index, raw in enumerate(data):
        if not isinstance(raw, dict):
            raise ParityAllowlistError(
                f"allowlist entry #{index} must be an object, "
                f"got {type(raw).__name__}"
            )
        keys = set(raw)
        if keys != _ALLOWLIST_ENTRY_FIELDS:
            unknown = sorted(keys - _ALLOWLIST_ENTRY_FIELDS)
            missing = sorted(_ALLOWLIST_ENTRY_FIELDS - keys)
            raise ParityAllowlistError(
                f"allowlist entry #{index} field mismatch: "
                f"unknown={unknown} missing={missing}"
            )
        entries.append(AllowlistEntry(**raw))
    return Allowlist(entries)
