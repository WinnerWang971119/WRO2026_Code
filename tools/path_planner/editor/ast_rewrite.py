"""LibCST-based source rewriter for ``field_data.EDGES`` node renames.

When the live field editor commits a node rename (e.g. ``"P1"`` to ``"P2"``),
the change must propagate back to ``tools/path_planner/field_data.py`` so the
next pipeline build sees the new name. The file is hand-authored Python with
extensive block comments that document each ``EDGES`` entry; those comments
are load-bearing for human readers, so we cannot round-trip the file through
``ast.parse`` + ``ast.unparse`` (which would discard them).

This module performs a surgical rewrite using `LibCST
<https://libcst.readthedocs.io/>`__, a concrete-syntax-tree library that
preserves comments, blank lines, indentation, and the quote style of every
string literal by default. Line endings are normalised to ``\n`` on write
(see :func:`rewrite_edges_uv`) so the on-disk output is byte-stable across
Windows / POSIX; an input file with CRLF endings is therefore re-emitted
with LF endings. The transformer only touches ``SimpleString`` value nodes
that:

1. Sit inside the top-level ``EDGES = [...]`` assignment, AND
2. Are the value of a ``"u"`` or ``"v"`` dict key, AND
3. Match one of the rename keys exactly (after quote stripping).

Everything else in the file -- including dicts elsewhere in the module that
happen to have ``"u"`` / ``"v"`` keys -- is left untouched.

In addition to the rename pass, this module can append or remove whole
``EDGE_PROJECTION`` dict entries in the same ``EDGES`` list (see
:func:`append_projection_edge` / :func:`remove_projection_edge`). Those
mutations honour the same scoping and byte-stability contract: they only
touch the top-level ``EDGES`` assignment, never rewrite the file when the
tree is unchanged, and normalise line endings to ``\n`` on write.

Public API: :func:`rewrite_edges_uv`, :func:`append_projection_edge`,
:func:`remove_projection_edge`.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, Optional, Union

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
    with the corresponding mapped name. Comments, blank lines, whitespace,
    and the quote style of unchanged literals survive the round-trip
    unchanged. Line endings are normalised: ``Path.read_text`` translates
    any CRLF input to LF before parsing, and the file is written back with
    ``newline="\n"``, so a CRLF source file is re-emitted with LF endings.

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


# ---------------------------------------------------------------------------
# Projection-edge append / remove
#
# These two operations add or delete a whole ``EDGE_PROJECTION`` dict entry in
# the same top-level ``EDGES`` list that ``rewrite_edges_uv`` renames into.
# They share the byte-stability + no-op-preserves-mtime contract: the file is
# written ONLY when the tree actually changes, and the output is normalised to
# LF line endings. All formatting (4-space list indent, 8-space dict-key
# indent, trailing commas, the closing bracket on its own line at column 0) is
# emitted explicitly so the result is byte-stable and matches the documented
# demo entry in ``field_data.py``.
# ---------------------------------------------------------------------------

def _parenthesized_indent(indent: str) -> cst.ParenthesizedWhitespace:
    """Build a ``ParenthesizedWhitespace`` that ends a line and re-indents.

    The returned whitespace renders as a single newline followed by
    ``indent`` spaces on the next line, with no trailing whitespace and no
    blank lines. It is the building block for nesting a list element, a dict
    element, or a closing bracket onto its own line.

    :param indent: The leading whitespace for the continuation line (for
        example ``"    "`` for a list element or ``""`` for a closing bracket
        at column 0).
    :return: A ``ParenthesizedWhitespace`` node with the requested indent.
    """
    return cst.ParenthesizedWhitespace(
        first_line=cst.TrailingWhitespace(
            whitespace=cst.SimpleWhitespace(""),
            comment=None,
            newline=cst.Newline(),
        ),
        empty_lines=[],
        indent=True,
        last_line=cst.SimpleWhitespace(indent),
    )


def _trailing_comma(after_indent: str) -> cst.Comma:
    """Build a trailing ``Comma`` whose successor sits on the next line.

    :param after_indent: The indent of the line that follows the comma --
        the indent of the next dict key, or of the closing brace.
    :return: A ``Comma`` node that emits ``,`` then a newline + ``after_indent``.
    """
    return cst.Comma(
        whitespace_before=cst.SimpleWhitespace(""),
        whitespace_after=_parenthesized_indent(after_indent),
    )


def _relink_into_element(
    whitespace: cst.BaseParenthesizableWhitespace, indent: str
) -> cst.ParenthesizedWhitespace:
    """Re-point trailing whitespace at a new element while keeping its comments.

    A list's ``lbracket.whitespace_after`` (for an empty list) or an element's
    ``comma.whitespace_after`` (for a non-empty list) can carry the comments
    that document the ``EDGES`` block -- LibCST attaches them as ``EmptyLine``
    nodes inside a ``ParenthesizedWhitespace``. When a new element is grafted
    in after that whitespace, we must keep those comment lines and only change
    the final indent so the new element nests correctly. A plain
    ``SimpleWhitespace`` carries no comments, so it is replaced wholesale.

    :param whitespace: The existing ``whitespace_after`` node to re-point.
    :param indent: The indent of the line the new element begins on (``"    "``
        for a top-level ``EDGES`` element).
    :return: A ``ParenthesizedWhitespace`` preserving any comment lines, with
        its ``last_line`` set to ``indent``.
    """
    if isinstance(whitespace, cst.ParenthesizedWhitespace):
        return whitespace.with_changes(last_line=cst.SimpleWhitespace(indent))
    return _parenthesized_indent(indent)


def _float_literal(value: float) -> cst.BaseExpression:
    """Build a float-literal node, wrapping negatives in a unary minus.

    ``cst.Float`` only accepts an unsigned magnitude -- in Python's grammar a
    negative number is a ``UnaryOperation`` over a positive literal, not a
    signed literal. This helper emits ``cst.Float`` for non-negative values
    and ``-<Float>`` for negative ones, so headings such as ``-90.0`` render
    correctly. ``repr(float(...))`` normalises integer inputs (``45`` ->
    ``45.0``) and produces a literal LibCST will round-trip.

    :param value: The numeric value to encode.
    :return: A ``cst.Float`` or a ``cst.UnaryOperation`` wrapping one.
    """
    magnitude = cst.Float(repr(abs(float(value))))
    if value < 0:
        return cst.UnaryOperation(operator=cst.Minus(), expression=magnitude)
    return magnitude


def _projection_dict(
    u: str, v: str, distance_cm: float, heading_deg: float
) -> cst.Dict:
    """Build the ``EDGE_PROJECTION`` dict node with canonical formatting.

    ``type`` is emitted as a bare ``cst.Name`` (``EDGE_PROJECTION``) because
    ``field_data.py`` imports that name; it must NOT be quoted. ``u`` and
    ``v`` are double-quoted string literals; ``distance_cm`` and
    ``heading_deg`` are float literals (``repr(float(...))`` so an integer
    input like ``45`` still renders as ``45.0``). The whitespace after each
    colon is set so the values align under the demo entry's columns, and the
    last element's comma leads the closing brace onto its own line.

    :param u: Source node name (the ``"u"`` value).
    :param v: Destination node name (the ``"v"`` value).
    :param distance_cm: Edge length in centimetres (the ``"distance_cm"`` value).
    :param heading_deg: Edge heading in degrees (the ``"heading_deg"`` value).
    :return: A formatted ``cst.Dict`` node for one projection entry.
    """
    key_indent = "        "  # 8 spaces -- dict keys sit two levels in.
    brace_indent = "    "  # 4 spaces -- closing brace aligns with the list element.
    elements = [
        cst.DictElement(
            key=cst.SimpleString('"type"'),
            value=cst.Name("EDGE_PROJECTION"),
            whitespace_after_colon=cst.SimpleWhitespace("         "),
            comma=_trailing_comma(key_indent),
        ),
        cst.DictElement(
            key=cst.SimpleString('"u"'),
            value=cst.SimpleString(f'"{u}"'),
            whitespace_after_colon=cst.SimpleWhitespace("            "),
            comma=_trailing_comma(key_indent),
        ),
        cst.DictElement(
            key=cst.SimpleString('"v"'),
            value=cst.SimpleString(f'"{v}"'),
            whitespace_after_colon=cst.SimpleWhitespace("            "),
            comma=_trailing_comma(key_indent),
        ),
        cst.DictElement(
            key=cst.SimpleString('"distance_cm"'),
            value=_float_literal(distance_cm),
            whitespace_after_colon=cst.SimpleWhitespace("  "),
            comma=_trailing_comma(key_indent),
        ),
        cst.DictElement(
            key=cst.SimpleString('"heading_deg"'),
            value=_float_literal(heading_deg),
            whitespace_after_colon=cst.SimpleWhitespace("  "),
            comma=_trailing_comma(brace_indent),
        ),
    ]
    return cst.Dict(
        elements=elements,
        lbrace=cst.LeftCurlyBrace(whitespace_after=_parenthesized_indent(key_indent)),
        rbrace=cst.RightCurlyBrace(whitespace_before=cst.SimpleWhitespace("")),
    )


def _projection_element(
    u: str, v: str, distance_cm: float, heading_deg: float
) -> cst.Element:
    """Wrap :func:`_projection_dict` as a trailing-comma list ``Element``.

    The element's own trailing comma carries no whitespace after it; the
    newline before the closing ``]`` is owned by the list's ``rbracket`` so
    that the appended element is always the last one and the bracket lands on
    its own line at column 0.

    :param u: Source node name.
    :param v: Destination node name.
    :param distance_cm: Edge length in centimetres.
    :param heading_deg: Edge heading in degrees.
    :return: A ``cst.Element`` ready to append to the ``EDGES`` list.
    """
    return cst.Element(
        value=_projection_dict(u, v, distance_cm, heading_deg),
        comma=cst.Comma(
            whitespace_before=cst.SimpleWhitespace(""),
            whitespace_after=cst.SimpleWhitespace(""),
        ),
    )


def _element_uv(element: cst.BaseElement) -> Optional[tuple[str, str]]:
    """Extract the ``(u, v)`` pair from a list element if it is a dict edge.

    Reads the ``"u"`` and ``"v"`` string values out of the element's
    ``cst.Dict`` value, stripping quotes via :func:`_strip_quotes`. Elements
    that are not dicts, or dicts missing a string ``"u"`` or ``"v"``, yield
    ``None`` so they are never matched by an append-dedup or remove scan.

    :param element: A list element from the ``EDGES`` assignment.
    :return: ``(u, v)`` if both keys are present as string literals, else
        ``None``.
    """
    value = element.value
    if not isinstance(value, cst.Dict):
        return None
    found_u: Optional[str] = None
    found_v: Optional[str] = None
    for dict_element in value.elements:
        if not isinstance(dict_element, cst.DictElement):
            continue
        key = dict_element.key
        item = dict_element.value
        if not isinstance(key, cst.SimpleString) or not isinstance(item, cst.SimpleString):
            continue
        key_text = _strip_quotes(key.value)
        if key_text == "u":
            found_u = _strip_quotes(item.value)
        elif key_text == "v":
            found_v = _strip_quotes(item.value)
    if found_u is None or found_v is None:
        return None
    return (found_u, found_v)


class _ProjectionAppender(cst.CSTTransformer):
    """Append one projection ``Element`` to the top-level ``EDGES`` list.

    Mirrors :class:`_EdgesUVRenamer`'s scoping, but gates on node identity
    rather than a depth counter: :meth:`visit_Assign` stashes the exact
    ``cst.List`` object that is the ``EDGES = [...]`` value, and
    :meth:`leave_List` acts only on that one node. A depth counter would also
    fire for a list nested as the value of a key inside an ``EDGES`` entry's
    dict (a future field such as ``"waypoints": [...]``), grafting the new
    element into the wrong list; identity matching rules that out. If an
    existing element already has the same ``(u, v)`` pair the append is
    skipped and :attr:`changed` stays ``False`` (no-op).

    :ivar changed: ``True`` once an element has been appended.
    :ivar _edges_list_node: The ``cst.List`` that is the ``EDGES`` value, or
        ``None`` when the traversal is not inside that assignment.
    """

    def __init__(
        self, u: str, v: str, distance_cm: float, heading_deg: float
    ) -> None:
        super().__init__()
        self._u = u
        self._v = v
        self._distance_cm = distance_cm
        self._heading_deg = heading_deg
        self._edges_list_node: Optional[cst.List] = None
        self.changed = False

    def visit_Assign(self, node: cst.Assign) -> None:
        """Stash the ``EDGES`` value list when its assignment is entered.

        Only a ``cst.List`` value is stashed; if the source ever wrote
        ``EDGES`` as a non-list expression there is nothing to append into.
        """
        if _EdgesUVRenamer._is_edges_assignment(node) and isinstance(
            node.value, cst.List
        ):
            self._edges_list_node = node.value

    def leave_Assign(
        self, original_node: cst.Assign, updated_node: cst.Assign
    ) -> cst.Assign:
        """Clear the stashed ``EDGES`` list when its assignment closes."""
        if _EdgesUVRenamer._is_edges_assignment(original_node):
            self._edges_list_node = None
        return updated_node

    def leave_List(
        self, original_node: cst.List, updated_node: cst.List
    ) -> cst.List:
        """Append the projection element to the EDGES list if not a duplicate.

        Only the list that IS the ``EDGES`` value is touched -- matched by
        object identity against the node stashed in :meth:`visit_Assign`, so a
        list nested as a dict value inside an ``EDGES`` entry is never grafted
        into. A duplicate ``(u, v)`` is a no-op. The previous last element's
        comma, or -- for an empty list -- the opening bracket, is re-indented
        so the new element nests at the list's 4-space indent while any
        documentation comments those whitespace nodes carry are preserved; the
        closing bracket is placed on its own line at column 0.

        :param original_node: The list as parsed.
        :param updated_node: The list after deeper passes (identical here).
        :return: The list with the new element appended, or unchanged.
        """
        if original_node is not self._edges_list_node:
            return updated_node

        elements = list(updated_node.elements)
        target = (self._u, self._v)
        for element in elements:
            if _element_uv(element) == target:
                return updated_node  # Duplicate -- no-op preserves mtime.

        new_element = _projection_element(
            self._u, self._v, self._distance_cm, self._heading_deg
        )
        result = updated_node
        if elements:
            # Re-point the previous last element's comma at the new element,
            # keeping any comments the comma whitespace carried.
            previous = elements[-1]
            if isinstance(previous.comma, cst.Comma):
                relinked_comma = previous.comma.with_changes(
                    whitespace_after=_relink_into_element(
                        previous.comma.whitespace_after, "    "
                    )
                )
            else:
                relinked_comma = _trailing_comma("    ")
            elements[-1] = previous.with_changes(comma=relinked_comma)
        else:
            # Empty list: the opening bracket's trailing whitespace holds the
            # block's documentation comments -- keep them, just re-indent into
            # the new element.
            result = result.with_changes(
                lbracket=cst.LeftSquareBracket(
                    whitespace_after=_relink_into_element(
                        result.lbracket.whitespace_after, "    "
                    )
                )
            )
        # The new element's own trailing comma carries no whitespace, so the
        # newline before ``]`` is owned here; preserve any pre-bracket comment.
        result = result.with_changes(
            rbracket=cst.RightSquareBracket(
                whitespace_before=_relink_into_element(
                    result.rbracket.whitespace_before, ""
                )
            ),
            elements=elements + [new_element],
        )
        self.changed = True
        return result


class _ProjectionRemover(cst.CSTTransformer):
    """Remove the first matching projection ``Element`` from ``EDGES``.

    Scans the ``EDGES`` list -- and only that list, matched by object identity
    against the node stashed in :meth:`visit_Assign` -- for the first element
    whose ``"u"`` and ``"v"`` values equal the targets, and drops it. Identity
    matching (rather than a depth counter) means a list nested as the value of
    a key inside an ``EDGES`` entry's dict (a future field such as
    ``"waypoints": [...]``) is never scanned, so a ``(u, v)`` that exists only
    in such a nested list is not falsely matched and deleted. When the removed
    element was the list's last, the new last element's comma and the closing
    bracket are re-normalised so no blank line is left behind; when the list
    becomes empty it is reset to the canonical ``[\\n]`` shape.

    :ivar changed: ``True`` once an element has been removed.
    :ivar _edges_list_node: The ``cst.List`` that is the ``EDGES`` value, or
        ``None`` when the traversal is not inside that assignment.
    """

    def __init__(self, u: str, v: str) -> None:
        super().__init__()
        self._u = u
        self._v = v
        self._edges_list_node: Optional[cst.List] = None
        self.changed = False

    def visit_Assign(self, node: cst.Assign) -> None:
        """Stash the ``EDGES`` value list when its assignment is entered.

        Only a ``cst.List`` value is stashed; if the source ever wrote
        ``EDGES`` as a non-list expression there is nothing to remove from.
        """
        if _EdgesUVRenamer._is_edges_assignment(node) and isinstance(
            node.value, cst.List
        ):
            self._edges_list_node = node.value

    def leave_Assign(
        self, original_node: cst.Assign, updated_node: cst.Assign
    ) -> cst.Assign:
        """Clear the stashed ``EDGES`` list when its assignment closes."""
        if _EdgesUVRenamer._is_edges_assignment(original_node):
            self._edges_list_node = None
        return updated_node

    def leave_List(
        self, original_node: cst.List, updated_node: cst.List
    ) -> cst.List:
        """Drop the first ``(u, v)``-matching element from the EDGES list.

        Only the list that IS the ``EDGES`` value is touched -- matched by
        object identity against the node stashed in :meth:`visit_Assign`, so a
        list nested as a dict value inside an ``EDGES`` entry is never scanned.
        The ``self.changed`` guard short-circuits once a removal has happened,
        preserving the no-op/mtime contract on any later visit.

        :param original_node: The list as parsed.
        :param updated_node: The list after deeper passes (identical here).
        :return: The list with the matching element removed, or unchanged.
        """
        if original_node is not self._edges_list_node or self.changed:
            return updated_node

        elements = list(updated_node.elements)
        target = (self._u, self._v)
        for index, element in enumerate(elements):
            if _element_uv(element) != target:
                continue
            was_last = index == len(elements) - 1
            del elements[index]
            result = updated_node
            # The removed element's leading newline + indent is owned upstream:
            # by the previous element's comma (index > 0) or by the opening
            # bracket (index == 0). When the removed element was the list's
            # last, that upstream whitespace must now re-point at the closing
            # bracket (indent ""); comments it carries are preserved. When the
            # removed element was not last, the upstream already leads the
            # element that followed, so it is left untouched.
            indent = "" if was_last else "    "
            if index > 0:
                if was_last:
                    previous = elements[-1]
                    if isinstance(previous.comma, cst.Comma):
                        relinked = previous.comma.with_changes(
                            whitespace_after=_relink_into_element(
                                previous.comma.whitespace_after, indent
                            )
                        )
                        elements[-1] = previous.with_changes(comma=relinked)
            else:
                result = result.with_changes(
                    lbracket=cst.LeftSquareBracket(
                        whitespace_after=_relink_into_element(
                            result.lbracket.whitespace_after, indent
                        )
                    )
                )
            if was_last:
                # Removing the list's last element makes the upstream whitespace
                # (the prior element's comma, or the opening bracket) own the
                # single line break before ``]``. Drop any newline the closing
                # bracket carried so no stray blank line is left behind.
                result = result.with_changes(
                    rbracket=cst.RightSquareBracket(
                        whitespace_before=cst.SimpleWhitespace("")
                    )
                )
            self.changed = True
            return result.with_changes(elements=elements)

        return updated_node


def append_projection_edge(
    path: Path, u: str, v: str, distance_cm: float, heading_deg: float
) -> bool:
    """Append an ``EDGE_PROJECTION`` entry to ``field_data.py``'s ``EDGES`` list.

    Loads ``path``, parses it with LibCST, and appends a new dict element to
    the top-level ``EDGES = [...]`` assignment. The element has keys in the
    fixed order ``type, u, v, distance_cm, heading_deg``: ``type`` is the bare
    name reference ``EDGE_PROJECTION`` (NOT a quoted string -- ``field_data``
    imports the name); ``u`` / ``v`` are double-quoted strings; the two
    measurements are float literals. Comments and every other entry in the
    list are preserved unchanged. The commented-out demo entry in the shipped
    file is a comment, not a list element, so it is never treated as existing.

    The append is a no-op (returns ``False``, file untouched, mtime preserved)
    when an entry with the same ``(u, v)`` pair already exists, or when there
    is no top-level ``EDGES`` assignment at all. An empty ``EDGES = []`` list
    is appended into normally.

    :param path: Filesystem path to the Python source file (typically
        ``tools/path_planner/field_data.py``).
    :param u: Source node name for the projection edge.
    :param v: Destination node name for the projection edge.
    :param distance_cm: Edge length in centimetres; emitted as a float literal.
    :param heading_deg: Edge heading in degrees; emitted as a float literal.
    :return: ``True`` if the file was rewritten on disk; ``False`` otherwise.
    :raises ValueError: If ``u`` or ``v`` contains a quote or backslash, which
        :func:`_wrap_with_same_style`-style unescaped emission cannot encode.
    :raises OSError: If the file cannot be read or written.
    :raises libcst.ParserSyntaxError: If the source is not valid Python.
    :note: Only the top-level ``EDGES`` list is touched; ``NODES`` and any
        other list or dict in the module are left untouched.
    """
    _validate_rename("u", u)
    _validate_rename("v", v)

    source = path.read_text(encoding="utf-8")
    module = cst.parse_module(source)
    transformer = _ProjectionAppender(u, v, distance_cm, heading_deg)
    modified = module.visit(transformer)
    if not transformer.changed:
        return False

    path.write_text(modified.code, encoding="utf-8", newline="\n")
    return True


def remove_projection_edge(path: Path, u: str, v: str) -> bool:
    """Remove the first ``EDGES`` entry whose ``(u, v)`` match ``u`` / ``v``.

    Loads ``path``, parses it with LibCST, and deletes the first dict element
    of the top-level ``EDGES = [...]`` assignment whose ``"u"`` value equals
    ``u`` and ``"v"`` value equals ``v`` (string comparison after stripping
    quotes via :func:`_strip_quotes`). The edge ``type`` is not inspected, so
    a stray non-projection entry with the same ``(u, v)`` is matched too; in
    the editor's data model an ``(u, v)`` pair is unique within ``EDGES``.
    Surrounding comments and all other entries survive unchanged, and the
    list whitespace is re-normalised so no blank line is left behind. A
    removed element's OWN leading comment is retained in place: LibCST owns
    that comment in the upstream whitespace (the prior element's comma, or the
    opening bracket) and re-points it onto the following node, so it stays in
    the file rather than being deleted with the element.

    The removal is a no-op (returns ``False``, file untouched, mtime
    preserved) when no entry matches, or when there is no top-level ``EDGES``
    assignment at all. The commented-out demo entry is a comment, not a list
    element, so it is never matched.

    :param path: Filesystem path to the Python source file (typically
        ``tools/path_planner/field_data.py``).
    :param u: Source node name to match against each entry's ``"u"`` value.
    :param v: Destination node name to match against each entry's ``"v"`` value.
    :return: ``True`` if an entry was removed and the file rewritten; ``False``
        if nothing matched.
    :raises OSError: If the file cannot be read or written.
    :raises libcst.ParserSyntaxError: If the source is not valid Python.
    :note: Only the top-level ``EDGES`` list is touched; ``NODES`` and any
        other list or dict in the module are left untouched.
    """
    source = path.read_text(encoding="utf-8")
    module = cst.parse_module(source)
    transformer = _ProjectionRemover(u, v)
    modified = module.visit(transformer)
    if not transformer.changed:
        return False

    path.write_text(modified.code, encoding="utf-8", newline="\n")
    return True
