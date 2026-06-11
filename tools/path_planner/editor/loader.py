"""JSON I/O for the live field editor (Issue #10, T1).

This module defines the on-disk JSON representation of the field geometry
the live editor mutates and round-trips. It is the bridge between the
hand-authored Python source of truth (``tools/path_planner/field_data.py``)
and the editor's in-memory ``FieldModel``.

Schema
------

The JSON file is a single object whose top-level keys mirror, exactly,
the module-level attributes the existing
:func:`tools.path_planner.validator.validate` reads from ``field_data``::

    {
      "FIELD_MM":              [width_mm, height_mm],
      "MAP_PNG":               "<relative path string>",
      "ROBOT_RADIUS_CM":       <float>,
      "COST_BLIND_MULTIPLIER": <float>,
      "START_NODE":            "<node name>",
      "START_HEADING_DEG":     <float in [0, 360)>,
      "NODES":                 {"<name>": [x_cm, y_cm], ...},
      "NODE_TYPES":            {"<name>": "line" | "blind", ...},
      "LINE_SEGMENTS":         [["<a>", "<b>"], ...],
      "BLIND_EDGES":           [["<a>", "<b>"], ...],
      "pivot_turns":           [["<u>", "<v>", "<w>"], ...],
      "OBSTACLES":             [[x_min, y_min, x_max, y_max, "<kind>"], ...]
    }

``pivot_turns`` is **optional** (absent or ``[]`` means no pivots). Each
entry is a 3-element node-name array ``["<u>", "<v>", "<w>"]`` declaring a
single-wheel pivot on the turn that arrives at ``v`` from ``u`` and leaves
to ``w``. Same array-of-arrays shape as ``LINE_SEGMENTS`` / ``BLIND_EDGES``,
so the loader's tuple discipline (below) applies to it identically.

The rich ``EDGES`` table that lives in ``field_data.py`` is deliberately
**not** part of the JSON schema -- it carries enum references
(``LineMode.FAST``, ``TurnMode.PIVOT_LEFT``, ...) and free-form Python comments
that JSON cannot represent losslessly. The migration tool keeps that
block in ``field_data.py`` and uses LibCST to rewrite it in place when
the editor renames a referenced node (see Issue #10 plan T6).

Tuple-vs-list discipline
------------------------

JSON has no tuple type. The validator, however, checks
``isinstance(value, tuple)`` for several fields (``FIELD_MM``, every
``NODES`` value, every ``LINE_SEGMENTS`` / ``BLIND_EDGES`` /
``OBSTACLES`` entry). :func:`load_field_json` therefore converts the
relevant JSON arrays into tuples at parse time so the loaded dict can be
handed straight to ``validator.validate`` via the editor's module-proxy
shim.

Determinism contract
--------------------

:func:`dump_field_json` writes byte-deterministic output:

* Top-level keys appear in a fixed, documented order.
* ``NODES`` and ``NODE_TYPES`` keys are emitted alphabetically.
* ``json.dumps`` is invoked with ``indent=2``; numeric scalars retain
  their Python type (a ``float`` round-trips as ``10.0``, an ``int`` as
  ``10``), which is what the spec calls "floats render as floats."
* The file ends with a single trailing newline.

The round-trip ``dump(load(dump(x)))`` is byte-equal to ``dump(x)`` for
any ``x`` that itself came from :func:`load_field_json` (i.e. has the
canonical Python types).
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from tools.path_planner.validator import FieldDataError

# Sibling of ``field_data.py``. Resolved lazily so test code can pass an
# explicit path without touching the on-disk default.
_DEFAULT_JSON_PATH = Path(__file__).resolve().parent.parent / "field_data.json"

# Top-level key order the dump writer enforces. The loader does not care
# about input key order (json is order-preserving in Python 3.7+, but the
# spec is explicit about output order, not input order).
_TOP_LEVEL_KEY_ORDER: tuple[str, ...] = (
    "FIELD_MM",
    "MAP_PNG",
    "ROBOT_RADIUS_CM",
    "COST_BLIND_MULTIPLIER",
    "START_NODE",
    "START_HEADING_DEG",
    "NODES",
    "NODE_TYPES",
    "LINE_SEGMENTS",
    "BLIND_EDGES",
    "pivot_turns",
    "OBSTACLES",
)

# Top-level keys that are optional in the JSON schema: absent / empty is a
# valid field. The dump writer emits them only when present so a field that
# never authored one round-trips byte-identically (absent stays absent),
# while a field that did keeps it in the canonical slot above. ``pivot_turns``
# (per-turn single-wheel pivots) is the sole such key today.
_OPTIONAL_TOP_LEVEL_KEYS: frozenset[str] = frozenset({"pivot_turns"})


def load_field_json(path: Path | None = None) -> dict:
    """Read ``field_data.json`` from disk and return it as a Python dict.

    The returned dict has the exact key set the existing
    :func:`tools.path_planner.validator.validate` expects on a
    ``field_data`` module, with JSON arrays converted into tuples for
    every field the validator type-checks via ``isinstance(..., tuple)``.

    Conversions applied:

    * ``FIELD_MM`` -- list of two numbers -> ``(width_mm, height_mm)``.
    * ``NODES[name]`` -- list of two numbers -> ``(x_cm, y_cm)``.
    * ``LINE_SEGMENTS[i]`` -- list of two strings -> ``(a, b)``.
    * ``BLIND_EDGES[i]`` -- list of two strings -> ``(a, b)``.
    * ``pivot_turns[i]`` -- list of three strings -> ``(u, v, w)``.
    * ``OBSTACLES[i]`` -- list of five elements ->
      ``(x_min, y_min, x_max, y_max, kind)``.

    All other fields (scalars, ``NODE_TYPES`` dict, ``MAP_PNG`` string,
    ``START_NODE`` / ``START_HEADING_DEG``) are returned unchanged.

    This function is pure: it performs a single read of ``path``, builds
    a fresh dict from the parsed JSON, and returns it. No module-level
    state is mutated.

    :param path: Optional override for the JSON file location. When
        ``None`` (the default), reads ``tools/path_planner/field_data.json``
        sibling of ``field_data.py``.
    :return: A new dict with the field-data attributes.
    :raises FieldDataError: when the file does not exist, or when the
        file contents are not valid JSON. The original
        :class:`json.JSONDecodeError` is chained via ``raise ... from``
        so the offending line/column survives.
    """
    target = Path(path) if path is not None else _DEFAULT_JSON_PATH

    try:
        raw_text = target.read_text(encoding="utf-8")
    except FileNotFoundError as exc:
        raise FieldDataError(
            f"field_data.json not found at {target} -- "
            "run `edit_field.py --migrate` to generate it from field_data.py"
        ) from exc

    try:
        parsed = json.loads(raw_text)
    except json.JSONDecodeError as exc:
        raise FieldDataError(
            f"field_data.json at {target} is not valid JSON "
            f"(line {exc.lineno}, column {exc.colno}): {exc.msg}"
        ) from exc

    if not isinstance(parsed, dict):
        raise FieldDataError(
            f"field_data.json must be a JSON object at the top level; "
            f"got: {type(parsed).__name__}"
        )

    return _convert_arrays_to_tuples(parsed)


def dump_field_json(data: dict, path: Path) -> None:
    """Write ``data`` to ``path`` as a byte-deterministic JSON file.

    The output:

    * Uses 2-space indentation (``indent=2``).
    * Emits top-level keys in the fixed order documented in this
      module's schema section (``FIELD_MM`` first, ``OBSTACLES`` last).
    * Sorts the keys inside ``NODES`` and ``NODE_TYPES`` alphabetically.
    * Preserves list order for ``LINE_SEGMENTS``, ``BLIND_EDGES``,
      ``OBSTACLES``.
    * Retains numeric type: ``float`` values render as ``10.0``,
      ``int`` values as ``10``, matching the spec's "floats render as
      floats" requirement.
    * Ends with a single trailing newline.

    Tuples (which the loader produced for ``FIELD_MM``, node coords,
    edges, obstacles) are accepted: :mod:`json` serialises them as
    arrays identically to lists, so the dump is independent of which
    sequence type the caller passes in.

    :param data: Dict whose keys are a superset of
        :data:`_TOP_LEVEL_KEY_ORDER`. Keys outside that set are
        ignored to keep the output schema fixed.
    :param path: Destination file path. The parent directory must
        already exist (the migration tool creates it explicitly).
    :return: ``None``. The file at ``path`` is overwritten.
    :raises KeyError: when ``data`` is missing a required top-level key.
    """
    ordered = _build_ordered_payload(data)

    # ``json.dumps`` with ``indent=2`` uses default separators
    # ``(', ', ': ')``, but the JSON spec for indented output collapses
    # the item separator to ``,`` (the indent supplies the spacing).
    # Pass ``separators=(",", ": ")`` explicitly to lock the format and
    # protect the byte-equality contract against future stdlib changes.
    serialised = json.dumps(ordered, indent=2, separators=(",", ": "))

    path.write_text(serialised + "\n", encoding="utf-8", newline="\n")


def _convert_arrays_to_tuples(parsed: dict) -> dict:
    """Return a copy of ``parsed`` with the validator-checked fields as tuples.

    The conversion is shallow except where the validator descends:
    ``NODES`` values, ``LINE_SEGMENTS`` / ``BLIND_EDGES`` /
    ``pivot_turns`` / ``OBSTACLES`` entries, and the top-level ``FIELD_MM``
    pair.

    Fields that are absent in ``parsed`` are absent in the result -- this
    function is not a validator. Missing-key errors are surfaced
    downstream by :func:`tools.path_planner.validator.validate`.

    :param parsed: Dict freshly parsed from JSON.
    :return: A new dict with tuple conversions applied. Input is not
        mutated.
    """
    result: dict[str, Any] = dict(parsed)

    if "FIELD_MM" in result:
        field_mm = result["FIELD_MM"]
        if isinstance(field_mm, list) and len(field_mm) == 2:
            result["FIELD_MM"] = tuple(field_mm)

    if "NODES" in result and isinstance(result["NODES"], dict):
        result["NODES"] = {
            name: tuple(coord) if isinstance(coord, list) and len(coord) == 2 else coord
            for name, coord in result["NODES"].items()
        }

    if "LINE_SEGMENTS" in result and isinstance(result["LINE_SEGMENTS"], list):
        result["LINE_SEGMENTS"] = [
            tuple(edge) if isinstance(edge, list) and len(edge) == 2 else edge
            for edge in result["LINE_SEGMENTS"]
        ]

    if "BLIND_EDGES" in result and isinstance(result["BLIND_EDGES"], list):
        result["BLIND_EDGES"] = [
            tuple(edge) if isinstance(edge, list) and len(edge) == 2 else edge
            for edge in result["BLIND_EDGES"]
        ]

    if "pivot_turns" in result and isinstance(result["pivot_turns"], list):
        result["pivot_turns"] = [
            tuple(turn) if isinstance(turn, list) and len(turn) == 3 else turn
            for turn in result["pivot_turns"]
        ]

    if "OBSTACLES" in result and isinstance(result["OBSTACLES"], list):
        result["OBSTACLES"] = [
            tuple(obs) if isinstance(obs, list) and len(obs) == 5 else obs
            for obs in result["OBSTACLES"]
        ]

    return result


def _build_ordered_payload(data: dict) -> dict:
    """Return a new dict whose key insertion order matches the dump contract.

    Python's ``json.dumps`` honours dict insertion order when
    ``sort_keys`` is left at its default ``False``. This helper assembles
    a fresh dict in :data:`_TOP_LEVEL_KEY_ORDER` and, for ``NODES`` /
    ``NODE_TYPES``, in alphabetical key order. List members are passed
    through verbatim so caller-controlled ordering (``LINE_SEGMENTS``,
    ``BLIND_EDGES``, ``pivot_turns``, ``OBSTACLES``) is preserved.

    Keys in :data:`_OPTIONAL_TOP_LEVEL_KEYS` (currently ``pivot_turns``)
    are emitted only when present in ``data``: a field that never authored
    one is omitted entirely so ``dump(load(x)) == dump(x)`` holds for a
    pivot-free input that lacked the key on disk (absent stays absent).

    :param data: Source dict. Must contain every required key in
        :data:`_TOP_LEVEL_KEY_ORDER` (i.e. every key not in
        :data:`_OPTIONAL_TOP_LEVEL_KEYS`).
    :return: A new dict with the canonical key ordering.
    :raises KeyError: when a required top-level key is missing.
    """
    ordered: dict[str, Any] = {}
    for key in _TOP_LEVEL_KEY_ORDER:
        if key in _OPTIONAL_TOP_LEVEL_KEYS and key not in data:
            continue
        value = data[key]
        if key in ("NODES", "NODE_TYPES") and isinstance(value, dict):
            ordered[key] = {name: value[name] for name in sorted(value)}
        else:
            ordered[key] = value
    return ordered
