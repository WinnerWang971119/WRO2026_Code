"""LibCST-based source rewriter for ``field_data.EDGES`` node renames.

When the live field editor commits a node rename (e.g. ``"P1"`` to ``"P2"``),
the change must propagate back to ``tools/path_planner/field_data.py`` so the
next pipeline build sees the new name. The file is hand-authored Python with
extensive block comments that document each ``EDGES`` entry; those comments
are load-bearing for human readers, so we cannot round-trip the file through
``ast.parse`` + ``ast.unparse`` (which would discard them).

This module performs a surgical rewrite using `LibCST
<https://libcst.readthedocs.io/>`__, a concrete-syntax-tree library that
preserves every byte of formatting -- comments, blank lines, indentation,
and the quote style of every string literal -- by default. The transformer
only touches ``SimpleString`` value nodes that:

1. Sit inside the top-level ``EDGES = [...]`` assignment, AND
2. Are the value of a ``"u"`` or ``"v"`` dict key, AND
3. Match one of the rename keys exactly (after quote stripping).

Everything else in the file -- including dicts elsewhere in the module that
happen to have ``"u"`` / ``"v"`` keys -- is left untouched.

Public API: :func:`rewrite_edges_uv`.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, Union

import libcst as cst


def _strip_quotes(literal: str) -> str:
    '''Strip the surrounding quote characters from a Python string literal.

    Handles the four forms ``SimpleString`` can take inside ``EDGES`` entries:
    single-quoted, double-quoted, triple-single-quoted, and triple-double-quoted.
    String prefixes (``r``, ``b``, ``u``, ``f`` and combinations, in either
    case) are also stripped.

    :param literal: The raw ``cst.SimpleString.value`` text (quotes included).
    :return: The inner string content with prefix and quotes removed.
    '''
    # Skip any leading prefix characters (r, b, u, f, case-insensitive).
    i = 0
    while i < len(literal) and literal[i] not in ("'", '"'):
        i += 1
    body = literal[i:]
    # Triple-quoted forms come first so we don't mis-strip a triple as a single.
    for quote in ('"""', "'''", '"', "'"):
        if body.startswith(quote) and body.endswith(quote) and len(body) >= 2 * len(quote):
            return body[len(quote) : -len(quote)]
    return body


def _wrap_with_same_style(template: str, new_inner: str) -> str:
    """Build a new ``SimpleString`` literal that mimics ``template``'s style.

    Preserves the original prefix (``r``, ``b``, ``u``, ``f``) and quote
    flavour (single / double, single / triple). The rename source comes from
    the GUI and is always a bare identifier-like token, so no escaping of the
    inner content is required; callers must not pass a value that contains
    the quote character or a backslash.

    :param template: The original ``cst.SimpleString.value`` whose style we
        want to mirror.
    :param new_inner: The replacement string content (no quotes).
    :return: A literal suitable for ``cst.SimpleString(value=...)``.
    """
    i = 0
    while i < len(template) and template[i] not in ("'", '"'):
        i += 1
    prefix = template[:i]
    body = template[i:]
    for quote in ('"""', "'''", '"', "'"):
        if body.startswith(quote):
            return f"{prefix}{quote}{new_inner}{quote}"
    # Defensive fallback: should never hit because SimpleString always has quotes.
    return f'"{new_inner}"'


class _EdgesUVRenamer(cst.CSTTransformer):
    """Rewrite ``u`` / ``v`` string values inside the top-level ``EDGES`` list.

    The transformer maintains a depth counter that is non-zero only while the
    traversal is inside the ``EDGES = [...]`` assignment's value subtree. All
    string rewrites are gated on that counter, so dicts elsewhere in the
    module (e.g. ``NODES``, future hand-authored fixtures) are left alone.

    :ivar _renames: Mapping of old node name to new node name.
    :ivar _edges_depth: Depth counter -- >0 means we are inside the EDGES value.
    :ivar changed: Set to ``True`` the first time a literal is rewritten.
    """

    def __init__(self, renames: Dict[str, str]) -> None:
        super().__init__()
        self._renames: Dict[str, str] = renames
        self._edges_depth: int = 0
        self.changed: bool = False

    def visit_Assign(self, node: cst.Assign) -> None:
        """Enter the EDGES scope when we see the top-level ``EDGES = ...`` assignment.

        Matches a single-target assignment whose target is the bare ``Name``
        ``EDGES``. Augmented assignments (``EDGES += ...``) and annotated
        assignments (``EDGES: list = ...``) are intentionally NOT matched --
        the source file uses a plain assignment and we want a tight scope.

        :param node: The assignment node being visited.
        """
        if self._is_edges_assignment(node):
            self._edges_depth += 1

    def leave_Assign(
        self, original_node: cst.Assign, updated_node: cst.Assign
    ) -> cst.Assign:
        """Pop the EDGES scope when leaving the assignment that opened it.

        :param original_node: The assignment as parsed (used for matching).
        :param updated_node: The possibly-rewritten assignment.
        :return: ``updated_node`` unchanged; rewrites happen deeper in the tree.
        """
        if self._is_edges_assignment(original_node):
            self._edges_depth -= 1
        return updated_node

    def leave_DictElement(
        self, original_node: cst.DictElement, updated_node: cst.DictElement
    ) -> Union[cst.DictElement, cst.BaseDictElement]:
        """Rewrite ``{"u": "old"}`` / ``{"v": "old"}`` pairs in scope.

        Only acts when (a) we are inside the EDGES assignment, (b) the key is
        a ``SimpleString`` whose stripped content is exactly ``u`` or ``v``,
        (c) the value is a ``SimpleString``, and (d) the value's stripped
        content is a rename source. The replacement keeps the original
        value's quote style.

        :param original_node: The dict element as parsed.
        :param updated_node: The possibly-rewritten dict element from deeper
            transformer passes (always identical to ``original_node`` in
            practice because nothing else in this transformer rewrites
            descendants of a DictElement).
        :return: A new ``DictElement`` with the value swapped, or the
            unchanged ``updated_node``.
        """
        if self._edges_depth <= 0:
            return updated_node

        key = updated_node.key
        value = updated_node.value
        if not isinstance(key, cst.SimpleString) or not isinstance(value, cst.SimpleString):
            return updated_node

        key_text = _strip_quotes(key.value)
        if key_text not in ("u", "v"):
            return updated_node

        old_name = _strip_quotes(value.value)
        if old_name not in self._renames:
            return updated_node

        new_literal = _wrap_with_same_style(value.value, self._renames[old_name])
        self.changed = True
        return updated_node.with_changes(value=cst.SimpleString(value=new_literal))

    @staticmethod
    def _is_edges_assignment(node: cst.Assign) -> bool:
        """Return True if ``node`` is the single-target ``EDGES = ...`` assignment.

        :param node: The assignment node to test.
        :return: True when the assignment has exactly one target and that
            target is the bare ``Name`` ``EDGES``.
        """
        if len(node.targets) != 1:
            return False
        target = node.targets[0].target
        return isinstance(target, cst.Name) and target.value == "EDGES"


# Characters that would break ``_wrap_with_same_style`` because it does not
# escape the inner string -- a quote would close the literal early and a
# backslash would start an escape sequence in the emitted Python source.
_UNSAFE_RENAME_CHARS: tuple[str, ...] = ('"', "'", "\\")


def _validate_rename(label: str, value: str) -> None:
    """Reject rename names that ``_wrap_with_same_style`` cannot safely emit.

    The GUI is the only caller, and node names are bare identifier-like
    tokens; this is a defense-in-depth check so a malformed name fails
    loudly before any source file is written.

    :param label: ``"key"`` or ``"value"`` -- used in the error message.
    :param value: The candidate rename string.
    :raises ValueError: If ``value`` contains a quote or backslash.
    """
    if not isinstance(value, str):
        raise ValueError(
            f"rename {label} must be a string, got {type(value).__name__}"
        )
    for ch in _UNSAFE_RENAME_CHARS:
        if ch in value:
            raise ValueError(
                f"rename {label} {value!r} contains unsafe character {ch!r}; "
                "node names must not contain quotes or backslashes"
            )


def rewrite_edges_uv(path: Path, renames: Dict[str, str]) -> bool:
    """Rewrite ``u`` and ``v`` node-name string literals inside ``EDGES``.

    Loads ``path`` as Python source, parses it with LibCST, walks the
    top-level ``EDGES = [...]`` assignment, and replaces any ``"u"`` or
    ``"v"`` dict value whose stripped string matches a key of ``renames``
    with the corresponding mapped name. All comments, blank lines,
    whitespace, and the quote style of unchanged literals are preserved
    byte-for-byte.

    The file is rewritten ONLY when at least one literal actually changes;
    if no rewrites occur (no ``EDGES`` assignment, empty list, or no rename
    hits), the file is left untouched on disk so mtime is preserved.

    :param path: Filesystem path to the Python source file (typically
        ``tools/path_planner/field_data.py``).
    :param renames: Mapping of old node name to new node name. An empty
        mapping is a guaranteed no-op. Every key and value must be a
        bare string without quote characters or backslashes; violations
        raise ``ValueError`` before the file is touched.
    :return: True if the file was rewritten on disk; False otherwise.
    :raises ValueError: If any rename key or value contains a quote or
        backslash character.
    :raises OSError: If the file cannot be read or written.
    :raises libcst.ParserSyntaxError: If the source is not valid Python.
    """
    if not renames:
        return False

    for old, new in renames.items():
        _validate_rename("key", old)
        _validate_rename("value", new)

    source = path.read_text(encoding="utf-8")
    module = cst.parse_module(source)
    transformer = _EdgesUVRenamer(renames)
    modified = module.visit(transformer)
    if not transformer.changed:
        return False

    # Pin LF line endings so the file is byte-stable across Windows / POSIX --
    # matches ``migrate._write_source`` and avoids platform-dependent diffs.
    path.write_text(modified.code, encoding="utf-8", newline="\n")
    return True
