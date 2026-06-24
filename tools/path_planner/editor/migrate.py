"""One-shot migration from Python-literal ``field_data.py`` to JSON loader.

The live field editor (Issue #10) treats ``tools/path_planner/field_data.json``
as the canonical source of truth for the field geometry the operator edits in
the GUI.  Before the editor can launch, the legacy ``field_data.py`` -- which
hand-rolls ``NODES``, ``NODE_TYPES``, ``LINE_SEGMENTS``, etc. as Python
literals -- has to be ported across.  This module performs that port exactly
once via :func:`run`, which the ``edit_field.py --migrate`` CLI calls (T12).

Behaviour
---------

#. **Idempotency probe** -- the source file is scanned for the literal
   ``NODES = {`` (any amount of whitespace before the ``=`` is tolerated).
   Its absence signals "already migrated"; the function prints a status line
   and returns ``0`` without touching disk.
#. **Lazy import of ``field_data``** -- when migration is required, the
   module is imported so the live module-attribute values can be captured
   without re-implementing the parser.  The import is intentionally local
   to :func:`run` so importing this module never forces the field-data
   module to load.
#. **Sidecar JSON** -- the captured attributes are bundled into a dict that
   matches the JSON schema documented in
   :mod:`tools.path_planner.editor.loader` and written via
   :func:`tools.path_planner.editor.loader.dump_field_json`.  Geometry / scalar
   values are passed through verbatim so numeric types (``int`` vs ``float``)
   survive the round-trip and the loaded module attributes compare equal by
   ``repr`` to their pre-migration counterparts.  A legacy Python ``EDGES``
   literal is carried INTO the JSON ``"EDGES"`` key (ADR-047): each entry is
   reduced to an override-only string-tag dict ``{type, u, v, primitive_mode?,
   direction?, stop_at_line?}`` -- enum-instance ``primitive_mode`` overrides are
   converted to their member-name strings, the ``DEFAULT`` sentinel is dropped
   (absence already means inherit), and derived geometry (``distance_cm`` /
   ``heading_deg``) is dropped (it is re-derived from node coords downstream).
#. **Source rewrite** -- ``field_data.py`` is rebuilt from a generated header
   that re-exposes the JSON-loaded values under the original module-attribute
   names, including ``EDGES = _data.get("EDGES", [])``.  No Python ``EDGES`` tail
   is preserved: once EDGES is JSON-sourced, a Python literal would be a stale
   second source of truth, so the legacy block is migrated into the JSON and the
   regenerated module reads it back from there.

The function is exposed for both direct programmatic use (the editor's
``Save & Build`` flow does not invoke it, but tests do) and CLI use via
``edit_field.py --migrate``.
"""

from __future__ import annotations

import enum
import importlib
import re
import sys
from pathlib import Path
from typing import Any

from tools.path_planner.editor.loader import dump_field_json

# Absolute paths to the files this migration touches.  Computed once at
# import time so the unit tests can monkey-patch them when needed.
_FIELD_DATA_PY: Path = Path(__file__).resolve().parent.parent / "field_data.py"
_FIELD_DATA_JSON: Path = Path(__file__).resolve().parent.parent / "field_data.json"

# The fourteen module attributes the legacy ``field_data.py`` exposes that the
# JSON sidecar must capture.  Order matches the schema's documented top-level
# key order; ``loader.dump_field_json`` re-orders these as needed.
_ATTRS_TO_CAPTURE: tuple[str, ...] = (
    "FIELD_MM",
    "MAP_PNG",
    "ROBOT_RADIUS_CM",
    "BLIND_DECAY_ENABLED",
    "BLIND_DECAY_K",
    "BLIND_DECAY_EXPONENT",
    "BLIND_DECAY_D_REF",
    "START_NODE",
    "START_HEADING_DEG",
    "NODES",
    "NODE_TYPES",
    "LINE_SEGMENTS",
    "BLIND_EDGES",
    "OBSTACLES",
)

# Regex that detects the geometry-literal section. ``re.MULTILINE`` lets the
# anchor float across the file; the optional whitespace before ``=`` matches
# the project's column-aligned style used in the generated header.
_NODES_LITERAL_RE = re.compile(r"^\s*NODES\s*=\s*\{", re.MULTILINE)

# Exit codes per the plan's CLI contract.
_EXIT_OK = 0
_EXIT_ERROR = 2

# Status messages printed on the happy paths.  Kept as module-level constants
# so tests can assert against them without duplicating the string body.
_MSG_ALREADY_MIGRATED = "field_data.py already migrated; field_data.json untouched"
_MSG_MIGRATED_FMT = "Wrote {json_path} and rewrote {py_path}"
_MSG_DRY_RUN_PREFIX = "[dry-run] "


def run(*, dry_run: bool = False) -> int:
    """Migrate ``field_data.py`` to the JSON-loader layout.

    The function is idempotent: a second invocation after a successful first
    run detects the migrated state and exits early without touching disk.

    :param dry_run: When ``True``, print what would be written but make no
        on-disk changes.  Defaults to ``False`` (perform the migration).
    :return: ``0`` on success or no-op, ``2`` on any error (file missing,
        parse failure, attribute capture failure, write failure).
    """
    try:
        source = _read_source(_FIELD_DATA_PY)
    except FileNotFoundError:
        _stderr(f"error: {_FIELD_DATA_PY} not found")
        return _EXIT_ERROR
    except OSError as exc:
        _stderr(f"error: failed to read {_FIELD_DATA_PY}: {exc}")
        return _EXIT_ERROR

    if not _NODES_LITERAL_RE.search(source):
        print(_MSG_ALREADY_MIGRATED)
        return _EXIT_OK

    try:
        captured = _capture_module_attributes()
    except Exception as exc:  # noqa: BLE001 -- surface any import error
        _stderr(f"error: failed to import field_data: {exc}")
        return _EXIT_ERROR

    try:
        docstring = _extract_module_docstring(source)
    except _MigrationError as exc:
        _stderr(f"error: {exc}")
        return _EXIT_ERROR

    new_source = _build_rewritten_source(docstring)
    json_payload = _build_json_payload(captured)

    if dry_run:
        print(
            f"{_MSG_DRY_RUN_PREFIX}"
            + _MSG_MIGRATED_FMT.format(
                json_path=_FIELD_DATA_JSON, py_path=_FIELD_DATA_PY
            )
        )
        return _EXIT_OK

    try:
        dump_field_json(json_payload, _FIELD_DATA_JSON)
    except OSError as exc:
        _stderr(f"error: failed to write {_FIELD_DATA_JSON}: {exc}")
        return _EXIT_ERROR

    try:
        _write_source(_FIELD_DATA_PY, new_source)
    except OSError as exc:
        _stderr(f"error: failed to write {_FIELD_DATA_PY}: {exc}")
        return _EXIT_ERROR

    print(_MSG_MIGRATED_FMT.format(json_path=_FIELD_DATA_JSON, py_path=_FIELD_DATA_PY))
    return _EXIT_OK


class _MigrationError(RuntimeError):
    """Raised by helpers when the source file cannot be split cleanly."""


def _read_source(path: Path) -> str:
    """Read ``path`` as UTF-8 and normalise line endings to ``\\n``.

    :meth:`pathlib.Path.read_text` uses universal-newlines mode, which
    translates any CRLF or CR sequences in the file into ``\\n`` before
    returning the string. The on-disk line endings are therefore NOT
    preserved through this round-trip; the caller pairs this helper with
    :func:`_write_source` (``newline='\\n'``) so the rewritten file is
    byte-stable on LF regardless of the source's original endings.

    :param path: Source file path.
    :return: Full file contents with ``\\n`` line endings.
    """
    return path.read_text(encoding="utf-8")


def _write_source(path: Path, contents: str) -> None:
    """Write ``contents`` to ``path`` with deterministic ``\\n`` line endings.

    Forcing ``newline='\\n'`` keeps the rewritten file's line endings stable
    across Windows / POSIX so downstream byte-comparison tests (AC1) are not
    confused by CRLF translation.

    :param path: Destination file path.
    :param contents: Full file contents to write.
    """
    path.write_text(contents, encoding="utf-8", newline="\n")


def _capture_module_attributes() -> dict[str, Any]:
    """Import ``tools.path_planner.field_data`` and snapshot its public attrs.

    The import happens here -- not at the top of this module -- so that
    importing :mod:`tools.path_planner.editor.migrate` never forces the
    field-data module to load.  If the module was already imported earlier
    in the process, ``importlib.reload`` ensures we read the current
    on-disk values rather than a stale cached version.

    :return: Dict mapping each attribute name in :data:`_ATTRS_TO_CAPTURE`
        to its current value.  Numeric types are not coerced.
    :raises AttributeError: If any expected attribute is missing.
    """
    module_name = "tools.path_planner.field_data"
    if module_name in sys.modules:
        module = importlib.reload(sys.modules[module_name])
    else:
        module = importlib.import_module(module_name)

    captured: dict[str, Any] = {}
    for attr in _ATTRS_TO_CAPTURE:
        captured[attr] = getattr(module, attr)
    # ``PIVOT_TURNS`` is optional: it predates the per-turn pivot model, so an
    # older field_data.py may not expose it. Capture it under its lowercase
    # JSON key only when present so a pivot-free migration omits the key
    # entirely (dump_field_json drops absent optional keys).
    pivot_turns = getattr(module, "PIVOT_TURNS", None)
    if pivot_turns is not None:
        captured["pivot_turns"] = pivot_turns
    # ``RESET_NODES`` is optional (absent / empty means no reset nodes). Capture
    # it under its uppercase JSON key when non-empty so a reset-free migration
    # omits the key entirely (dump_field_json drops absent optional keys).
    reset_nodes = getattr(module, "RESET_NODES", None)
    if reset_nodes:
        captured["RESET_NODES"] = reset_nodes
    # ``BLIND_PROJECTION_AUTO_SEEK`` is optional with a default of ``True``
    # (absent == True). The JSON only carries it when explicitly ``False``
    # (the loader comment: "emitted only when set to its non-default False").
    # Capture the lowercase key only when the legacy module sets it to False
    # so a default-True field round-trips without a spurious JSON key.
    blind_proj_auto_seek = getattr(module, "BLIND_PROJECTION_AUTO_SEEK", None)
    if blind_proj_auto_seek is False:
        captured["blind_projection_auto_seek"] = False
    # A legacy Python ``EDGES`` literal (enum-instance overrides) is carried into
    # the JSON ``"EDGES"`` key as override-only string-tag dicts (ADR-047). An
    # empty list contributes nothing -- omit the key so a field with no overrides
    # round-trips without an empty ``"EDGES": []`` (dump_field_json drops absent
    # optional keys, matching the pivot-free path above).
    edges = getattr(module, "EDGES", None)
    if edges:
        captured["EDGES"] = [_edge_to_override_dict(entry) for entry in edges]
    return captured


# Geometry keys derived from node coords downstream -- dropped on migration so
# the JSON carries override-only overlay entries (no stale pinned geometry).
_DERIVED_GEOMETRY_KEYS: frozenset[str] = frozenset({"distance_cm", "heading_deg"})

# Required identity keys on every EDGES entry; copied verbatim.
_EDGE_REQUIRED_KEYS: tuple[str, ...] = ("type", "u", "v")

# Optional per-edge override keys carried into the JSON (string tags). Geometry
# is intentionally NOT here -- it is derived, not migrated.
_EDGE_OVERRIDE_KEYS: tuple[str, ...] = ("primitive_mode", "direction", "stop_at_line")


def _edge_to_override_dict(entry: dict[str, Any]) -> dict[str, Any]:
    """Reduce a legacy ``EDGES`` entry to an override-only JSON string-tag dict.

    Carries the required identity keys (``type`` / ``u`` / ``v``) verbatim and
    the optional override keys (``primitive_mode`` / ``direction`` /
    ``stop_at_line``) when present, converting any enum-instance value to its
    member-name string (JSON cannot hold an enum). The ``DEFAULT`` sentinel is
    dropped -- absence already means "inherit the pipeline default", and the
    validator rejects an explicit ``"DEFAULT"`` tag. Derived geometry
    (``distance_cm`` / ``heading_deg``) is dropped: it is re-derived from node
    coords downstream, so pinning it here would only risk going stale when nodes
    move.

    :param entry: One dict from the legacy Python ``EDGES`` list.
    :return: A new override-only dict suitable for the JSON ``"EDGES"`` key.
    """
    override: dict[str, Any] = {}
    for key in _EDGE_REQUIRED_KEYS:
        if key in entry:
            override[key] = _enum_name(entry[key])
    for key in _EDGE_OVERRIDE_KEYS:
        if key not in entry:
            continue
        value = _enum_name(entry[key])
        # ``DEFAULT`` (sentinel) == absence: do not author it into the JSON.
        if isinstance(value, str) and value == "DEFAULT":
            continue
        override[key] = value
    # Any remaining non-geometry custom key is preserved verbatim so a
    # hand-authored entry is not silently truncated; geometry is dropped.
    for key, value in entry.items():
        if key in _EDGE_REQUIRED_KEYS or key in _EDGE_OVERRIDE_KEYS:
            continue
        if key in _DERIVED_GEOMETRY_KEYS:
            continue
        override[key] = _enum_name(value)
    return override


def _enum_name(value: Any) -> Any:
    """Return an enum value's member name, or the value unchanged otherwise.

    JSON cannot serialise an ``enum.Enum`` instance; the project encodes a
    per-edge ``primitive_mode`` override as its member-name string (e.g.
    ``LineMode.FAST`` -> ``"FAST"``). A value that is already a plain string or
    scalar passes through untouched.

    :param value: A field value from an ``EDGES`` entry.
    :return: ``value.name`` when ``value`` is an enum member, else ``value``.
    """
    if isinstance(value, enum.Enum):
        return value.name
    return value


def _build_json_payload(captured: dict[str, Any]) -> dict[str, Any]:
    """Pass ``captured`` straight through as the JSON payload.

    The values are forwarded verbatim -- no ``float()`` / ``int()`` casts --
    so ``FIELD_MM = (3000, 2000)`` round-trips as ``[3000, 2000]`` rather
    than ``[3000.0, 2000.0]``.  ``dump_field_json`` accepts tuples (it
    serialises them as JSON arrays) and re-orders the top-level keys, so
    this helper exists purely to express the intent: "no transformation."

    The optional ``pivot_turns`` and ``EDGES`` keys (present in ``captured``
    only when the source module exposed a non-empty ``PIVOT_TURNS`` / ``EDGES``)
    are forwarded the same way; ``dump_field_json`` drops them when absent, so a
    field with no pivots / no overrides produces JSON without those keys.

    :param captured: Result of :func:`_capture_module_attributes`.
    :return: A new dict with the same keys and values.
    """
    return dict(captured)


def _extract_module_docstring(source: str) -> str:
    """Return the module-level docstring verbatim (including the quote chars).

    The migration regenerates everything above ``EDGES`` from a template,
    but the docstring is hand-authored prose that must survive untouched.
    We locate it by walking ``source`` past any leading whitespace and
    matching the first triple-quoted string literal that follows.  Only
    triple-quoted forms are recognised because ``field_data.py`` (and every
    other module in the repo) uses the standard triple-double-quote
    convention.

    :param source: Full contents of ``field_data.py``.
    :return: The docstring exactly as it appears in the file, surrounding
        triple quotes included.
    :raises _MigrationError: When no module docstring is found.
    """
    # Skip optional leading whitespace / blank lines.
    index = 0
    while index < len(source) and source[index] in " \t\r\n":
        index += 1

    if index >= len(source):
        raise _MigrationError("field_data.py is empty; no docstring to capture")

    # Accept either triple-double-quote or triple-single-quote opener.
    for opener in ('"""', "'''"):
        if source.startswith(opener, index):
            closer_index = source.find(opener, index + len(opener))
            if closer_index == -1:
                raise _MigrationError(
                    "module docstring opener found but no matching closer"
                )
            return source[index : closer_index + len(opener)]

    raise _MigrationError(
        "field_data.py does not start with a triple-quoted module docstring"
    )


def _build_rewritten_source(docstring: str) -> str:
    """Assemble the migrated ``field_data.py`` from the JSON-loader template.

    The output is a fixed template: the captured docstring, the loader and
    enum imports, the ``_data = load_field_json()`` call, and the aliases that
    re-expose the JSON values under the original module-attribute names --
    including ``EDGES = _data.get("EDGES", [])``. No Python ``EDGES`` tail is
    appended: once EDGES is JSON-sourced (ADR-047), a Python literal would be a
    stale second source of truth, so the legacy block was migrated into the JSON
    by :func:`_capture_module_attributes` and the regenerated module reads it
    back from there.

    :param docstring: Verbatim docstring (quotes included) from the original
        file.
    :return: The full new file contents, ready to be written.
    """
    header_lines = [
        docstring,
        "",
        "from tools.path_planner.editor.loader import load_field_json",
        "from tools.path_planner.graph_builder import EDGE_BLIND, EDGE_LINE  # noqa: F401",
        "from tools.path_planner.pipeline_config import LineMode, BlindMode, TurnMode  # noqa: F401",
        "",
        "_data = load_field_json()",
        'FIELD_MM              = _data["FIELD_MM"]',
        'MAP_PNG               = _data["MAP_PNG"]',
        'ROBOT_RADIUS_CM       = _data["ROBOT_RADIUS_CM"]',
        'BLIND_DECAY_ENABLED   = _data["BLIND_DECAY_ENABLED"]',
        'BLIND_DECAY_K         = _data["BLIND_DECAY_K"]',
        'BLIND_DECAY_EXPONENT  = _data["BLIND_DECAY_EXPONENT"]',
        'BLIND_DECAY_D_REF     = _data["BLIND_DECAY_D_REF"]',
        'START_NODE            = _data["START_NODE"]',
        'START_HEADING_DEG     = _data["START_HEADING_DEG"]',
        'NODES                 = _data["NODES"]',
        'NODE_TYPES            = _data["NODE_TYPES"]',
        'LINE_SEGMENTS         = _data["LINE_SEGMENTS"]',
        'BLIND_EDGES           = _data["BLIND_EDGES"]',
        'OBSTACLES             = _data["OBSTACLES"]',
        'PIVOT_TURNS           = _data.get("pivot_turns", [])',
        'BLIND_PROJECTION_AUTO_SEEK = _data.get("blind_projection_auto_seek", True)',
        'RESET_NODES           = _data.get("RESET_NODES", {})',
        'EDGES                 = _data.get("EDGES", [])',
        "",
    ]
    return "\n".join(header_lines) + "\n"


def _stderr(message: str) -> None:
    """Write ``message`` plus a newline to ``sys.stderr``.

    Centralised so the CLI exit-path messages can be redirected or silenced
    in tests by replacing this single function.

    :param message: Text to emit on stderr (no trailing newline required).
    """
    print(message, file=sys.stderr)
