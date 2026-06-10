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
   :func:`tools.path_planner.editor.loader.dump_field_json`.  Values are
   passed through verbatim so numeric types (``int`` vs ``float``) survive
   the round-trip and the loaded module attributes compare equal by ``repr``
   to their pre-migration counterparts.
#. **Source rewrite** -- ``field_data.py`` is split into two pieces.  The
   header section (docstring + imports + geometry literals) is replaced by
   a generated header that re-exposes the JSON-loaded values under the
   original module-attribute names.  The tail section -- everything from
   the ``# ---------------------------------------------------------------------------``
   banner that introduces the rich ``EDGES`` table downward -- is preserved
   byte-for-byte verbatim.  The author's commentary inside ``EDGES`` (the
   ``# [0] PROJECTION demo --`` style notes) is load-bearing for future
   readers and must not be reformatted, reflowed, or reordered.

The function is exposed for both direct programmatic use (the editor's
``Save & Build`` flow does not invoke it, but tests do) and CLI use via
``edit_field.py --migrate``.
"""

from __future__ import annotations

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

# The eleven module attributes the legacy ``field_data.py`` exposes that the
# JSON sidecar must capture.  Order matches the schema's documented top-level
# key order; ``loader.dump_field_json`` re-orders these as needed.
_ATTRS_TO_CAPTURE: tuple[str, ...] = (
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
    "OBSTACLES",
)

# Regex that detects the geometry-literal section. ``re.MULTILINE`` lets the
# anchor float across the file; the optional whitespace before ``=`` matches
# the project's column-aligned style used in the generated header.
_NODES_LITERAL_RE = re.compile(r"^\s*NODES\s*=\s*\{", re.MULTILINE)

# Banner that introduces the rich ``EDGES`` block.  Captured verbatim from
# the live file so the dashed line is split out as the tail anchor.
_EDGES_BANNER = (
    "# ---------------------------------------------------------------------------\n"
    "# Rich edge list"
)

# Fallback anchor: the bare assignment line.  Used only when the banner is
# missing (the author edited the file).  Everything from the blank line
# preceding ``EDGES = [`` onward becomes the tail.
_EDGES_ASSIGN_TOKEN = "EDGES = ["

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
        tail = _extract_edges_tail(source)
    except _MigrationError as exc:
        _stderr(f"error: {exc}")
        return _EXIT_ERROR

    new_source = _build_rewritten_source(docstring, tail)
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
    return captured


def _build_json_payload(captured: dict[str, Any]) -> dict[str, Any]:
    """Pass ``captured`` straight through as the JSON payload.

    The values are forwarded verbatim -- no ``float()`` / ``int()`` casts --
    so ``FIELD_MM = (3000, 2000)`` round-trips as ``[3000, 2000]`` rather
    than ``[3000.0, 2000.0]``.  ``dump_field_json`` accepts tuples (it
    serialises them as JSON arrays) and re-orders the top-level keys, so
    this helper exists purely to express the intent: "no transformation."

    The optional ``pivot_turns`` key (present in ``captured`` only when the
    source module exposed ``PIVOT_TURNS``) is forwarded the same way;
    ``dump_field_json`` drops it when absent, so a pivot-free migration
    produces JSON without the key.

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


def _extract_edges_tail(source: str) -> str:
    """Return everything from the rich-EDGES banner downward, byte-for-byte.

    Strategy:

    #. Look for the banner string :data:`_EDGES_BANNER` (the dashed line
       plus the first words of the "Rich edge list" comment).  When found,
       walk backward to the start of the line that contains the dashes and
       take that as the tail boundary.
    #. If the banner is missing (the author edited it away), fall back to
       the first occurrence of ``EDGES = [`` and walk backward over the
       single preceding blank line so the generated header is separated
       from the table by exactly one blank line.

    The returned string is sliced out of ``source`` without modification so
    every comment, blank line, and trailing closing bracket survives.

    :param source: Full contents of ``field_data.py``.
    :return: The tail section, suitable for direct concatenation onto the
        generated header.
    :raises _MigrationError: When neither the banner nor the ``EDGES =``
        assignment can be located.
    """
    banner_index = source.find(_EDGES_BANNER)
    if banner_index != -1:
        # Walk back to the start of the line that contains the dashes.
        line_start = source.rfind("\n", 0, banner_index)
        if line_start == -1:
            return source
        return source[line_start + 1 :]

    assign_index = source.find(_EDGES_ASSIGN_TOKEN)
    if assign_index == -1:
        raise _MigrationError(
            "could not locate the EDGES banner or 'EDGES = [' assignment"
        )

    # Walk back to the start of the EDGES line, then over one preceding
    # blank line if present so the header ends on a single blank line.
    line_start = source.rfind("\n", 0, assign_index)
    if line_start == -1:
        return source
    cut = line_start + 1
    # Trim one leading blank line if present (the line just above ``EDGES =``).
    prev_nl = source.rfind("\n", 0, line_start)
    if prev_nl != -1 and source[prev_nl + 1 : line_start + 1].strip() == "":
        cut = prev_nl + 1
    return source[cut:]


def _build_rewritten_source(docstring: str, tail: str) -> str:
    """Assemble the migrated ``field_data.py`` from its two pieces.

    The header is a fixed template: the captured docstring, the loader and
    enum imports the rich-``EDGES`` table needs, the ``_data = load_field_json()``
    call, and the eleven aliases that re-expose the JSON values under the
    original module-attribute names.

    The tail is then appended, *normalised* so exactly one blank line
    separates the alias block from the tail's first non-blank line: any
    leading newlines on ``tail`` are stripped before joining, then the
    header's own trailing newline supplies the single separator.  This
    normalisation is the only modification applied to the tail; every
    non-blank character (including all comments inside ``EDGES``) is
    preserved byte-for-byte.

    :param docstring: Verbatim docstring (quotes included) from the original
        file.
    :param tail: Verbatim slice of the source from the rich-EDGES banner
        downward.  Leading blank lines are normalised away.
    :return: The full new file contents, ready to be written.
    """
    header_lines = [
        docstring,
        "",
        "from tools.path_planner.editor.loader import load_field_json",
        "from tools.path_planner.graph_builder import EDGE_BLIND, EDGE_LINE, EDGE_PROJECTION, Pivot  # noqa: F401",
        "from tools.path_planner.pipeline_config import LineMode, BlindMode, ProjectionMode, TurnMode  # noqa: F401",
        "",
        "_data = load_field_json()",
        'FIELD_MM              = _data["FIELD_MM"]',
        'MAP_PNG               = _data["MAP_PNG"]',
        'ROBOT_RADIUS_CM       = _data["ROBOT_RADIUS_CM"]',
        'COST_BLIND_MULTIPLIER = _data["COST_BLIND_MULTIPLIER"]',
        'START_NODE            = _data["START_NODE"]',
        'START_HEADING_DEG     = _data["START_HEADING_DEG"]',
        'NODES                 = _data["NODES"]',
        'NODE_TYPES            = _data["NODE_TYPES"]',
        'LINE_SEGMENTS         = _data["LINE_SEGMENTS"]',
        'BLIND_EDGES           = _data["BLIND_EDGES"]',
        'OBSTACLES             = _data["OBSTACLES"]',
        'PIVOT_TURNS           = _data.get("pivot_turns", [])',
        "",
    ]
    header = "\n".join(header_lines) + "\n"

    # Guarantee exactly one blank line between header and tail by stripping
    # any leading blank lines from the tail before joining.
    normalised_tail = tail.lstrip("\n")
    return header + normalised_tail


def _stderr(message: str) -> None:
    """Write ``message`` plus a newline to ``sys.stderr``.

    Centralised so the CLI exit-path messages can be redirected or silenced
    in tests by replacing this single function.

    :param message: Text to emit on stderr (no trailing newline required).
    """
    print(message, file=sys.stderr)
