"""Command objects that mutate a :class:`FieldModel` with undo support.

Every user-driven edit in the live field editor is funnelled through a
:class:`Command`.  Each command is a self-contained ``@dataclass`` that knows
both how to apply itself to a :class:`~tools.path_planner.editor.model.FieldModel`
and how to revert that mutation.  Inverse data needed for the revert (the
original coordinates of a moved node, the index a deleted obstacle lived at,
etc.) is captured during :meth:`Command.apply` and stored on the command itself
in private ``_underscored`` fields.  The :class:`~tools.path_planner.editor.history.History`
stack then only needs to keep a list of commands -- no separate snapshot is
required for unwinding.

Commands that detect an illegal pre-condition (duplicate edge, missing node,
out-of-range index, ...) raise the documented exception *before* mutating the
model.  ``History`` treats that as a no-op: the command is never pushed onto
the undo stack.

This module is pure logic with no GUI dependency: it can be exercised directly
from the test suite (T8) without instantiating Tk or any canvas widget.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol, runtime_checkable

from tools.path_planner.editor.model import FieldModel

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

#: Allowed values for ``FieldModel.node_types`` entries.
_VALID_NODE_TYPES: frozenset[str] = frozenset({"line", "blind"})

#: Allowed ``kind`` values for edge-batch entries.
_VALID_EDGE_KINDS: frozenset[str] = frozenset({"line", "blind"})

#: Heading bounds, in degrees, CCW from +x (matches ``field_data`` convention).
_HEADING_MIN_DEG: float = 0.0
_HEADING_MAX_DEG_EXCLUSIVE: float = 360.0


# ---------------------------------------------------------------------------
# Protocol
# ---------------------------------------------------------------------------


@runtime_checkable
class Command(Protocol):
    """Structural protocol every concrete editor command satisfies.

    A command is a reversible mutation of a :class:`FieldModel`.  The contract
    is intentionally minimal so that :class:`~tools.path_planner.editor.history.History`
    only has to know about two verbs.

    Implementations must guarantee that after a successful call to
    :meth:`apply` followed by :meth:`revert`, the model is byte-identical to
    its pre-:meth:`apply` state.  Tests TC4, TC13, and TC14 enforce this
    invariant.
    """

    def apply(self, model: FieldModel) -> None:
        """Apply this command's mutation to *model*.

        Implementations may capture inverse data on ``self`` here (see the
        ``_original_*`` / ``_deleted_*`` fields on the concrete commands).
        On pre-condition failure, raise the documented exception *before*
        mutating the model.

        :param model: The :class:`FieldModel` to mutate in place.
        :raises Exception: Subclass-specific; the model must be untouched if
            an exception is raised.
        """
        ...

    def revert(self, model: FieldModel) -> None:
        """Undo a previously successful :meth:`apply` on *model*.

        Must only be called once per successful :meth:`apply`, and must leave
        *model* in a state byte-identical to its pre-:meth:`apply` snapshot.

        :param model: The :class:`FieldModel` to restore.
        """
        ...


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _edge_equivalent(a: tuple[str, str], b: tuple[str, str]) -> bool:
    """Return ``True`` if *a* and *b* describe the same undirected edge.

    Edges in :class:`FieldModel` are stored as ordered 2-tuples, but the
    underlying graph is undirected: ``("A", "B")`` and ``("B", "A")`` denote
    the same connection.  This helper centralises that equivalence so every
    duplicate-edge check uses the same rule.

    :param a: First edge as ``(node_a, node_b)``.
    :param b: Second edge as ``(node_a, node_b)``.
    :return: ``True`` iff the two edges connect the same pair of nodes.
    """
    return a == b or (a[0] == b[1] and a[1] == b[0])


def _find_edge_index(
    edges: list[tuple[str, str]], target: tuple[str, str]
) -> int:
    """Return the index of *target* in *edges*, considering edges undirected.

    :param edges: The list to search (typically ``line_segments`` or
        ``blind_edges``).
    :param target: The edge to locate.
    :return: The first index ``i`` such that
        :func:`_edge_equivalent` ``(edges[i], target)`` is ``True``,
        or ``-1`` if no such index exists.
    """
    for index, existing in enumerate(edges):
        if _edge_equivalent(existing, target):
            return index
    return -1


# ---------------------------------------------------------------------------
# Node commands
# ---------------------------------------------------------------------------


@dataclass
class AddNode:
    """Insert a new node into the model.

    :ivar name: Name of the new node.  Must not already exist in
        ``model.nodes``.
    :ivar xy: ``(x_cm, y_cm)`` coordinates of the new node.
    :ivar node_type: Either ``"line"`` or ``"blind"``.
    """

    name: str
    xy: tuple[float, float]
    node_type: str

    def apply(self, model: FieldModel) -> None:
        """Insert ``self.name`` into ``model.nodes`` and ``model.node_types``.

        :param model: Target model.
        :raises ValueError: If *name* already exists in ``model.nodes`` or
            ``model.node_types``, or if *node_type* is not one of
            ``"line"`` / ``"blind"``.
        """
        if self.node_type not in _VALID_NODE_TYPES:
            raise ValueError(
                f"AddNode: node_type must be one of "
                f"{sorted(_VALID_NODE_TYPES)!r}, got {self.node_type!r}"
            )
        if self.name in model.nodes or self.name in model.node_types:
            raise ValueError(
                f"AddNode: a node named {self.name!r} already exists"
            )
        model.nodes[self.name] = self.xy
        model.node_types[self.name] = self.node_type

    def revert(self, model: FieldModel) -> None:
        """Remove the node previously inserted by :meth:`apply`.

        :param model: Target model.
        """
        # ``apply`` only succeeds after asserting both keys were absent, so a
        # plain ``del`` is correct here.  Defensive ``pop`` guards against a
        # caller manually mutating the dicts between apply and revert.
        model.nodes.pop(self.name, None)
        model.node_types.pop(self.name, None)


@dataclass
class MoveNode:
    """Change the coordinates of an existing node.

    :ivar name: Name of the node to move.
    :ivar new_xy: New ``(x_cm, y_cm)`` coordinates.
    """

    name: str
    new_xy: tuple[float, float]

    # Captured at apply time so revert can restore the prior coordinates.
    _original_xy: tuple[float, float] | None = field(
        default=None, init=False, repr=False
    )

    def apply(self, model: FieldModel) -> None:
        """Replace ``model.nodes[self.name]`` with ``self.new_xy``.

        :param model: Target model.
        :raises KeyError: If ``self.name`` is not in ``model.nodes``.
        """
        if self.name not in model.nodes:
            raise KeyError(
                f"MoveNode: no node named {self.name!r} in model.nodes"
            )
        self._original_xy = model.nodes[self.name]
        model.nodes[self.name] = self.new_xy

    def revert(self, model: FieldModel) -> None:
        """Restore the node to its pre-:meth:`apply` coordinates.

        :param model: Target model.
        """
        # ``apply`` populates ``_original_xy``; if revert is called without a
        # successful apply, this is a programming error and we want to fail
        # loudly rather than silently swallow it.
        assert self._original_xy is not None, "MoveNode.revert before apply"
        model.nodes[self.name] = self._original_xy


@dataclass
class RenameNode:
    """Rename a node, cascading the change through every container that
    references it.

    Updates ``model.nodes``, ``model.node_types``, every entry of
    ``model.line_segments`` and ``model.blind_edges`` that mentions the old
    name, and ``model.start_node`` if it equals the old name.

    :ivar old: Existing node name.
    :ivar new: Replacement node name.  Must not already exist.
    """

    old: str
    new: str

    # True iff ``model.start_node == self.old`` at apply time, so revert knows
    # whether to restore ``start_node``.  No other state needs to be captured:
    # the cascade is fully determined by ``self.old`` and ``self.new``.
    _start_node_was_old: bool = field(default=False, init=False, repr=False)

    def apply(self, model: FieldModel) -> None:
        """Rename ``self.old`` to ``self.new`` everywhere in *model*.

        :param model: Target model.
        :raises KeyError: If ``self.old`` is not present in ``model.nodes``.
        :raises ValueError: If ``self.new`` already exists in ``model.nodes``,
            or if ``self.old`` equals ``self.new``.
        """
        if self.old == self.new:
            raise ValueError(
                "RenameNode: new name must differ from old name"
            )
        if self.old not in model.nodes:
            raise KeyError(
                f"RenameNode: no node named {self.old!r} in model.nodes"
            )
        if self.new in model.nodes:
            raise ValueError(
                f"RenameNode: a node named {self.new!r} already exists"
            )

        # Rebuild nodes / node_types in place to preserve insertion order
        # where possible.  A simple pop + insert would push the renamed node
        # to the end, which would change ``to_json_dict`` output ordering for
        # any caller relying on it.  Sorted output (the JSON writer's
        # responsibility) makes order moot for the saved file, but in-memory
        # ordering still matters for deterministic tests.
        model.nodes = {
            (self.new if name == self.old else name): xy
            for name, xy in model.nodes.items()
        }
        model.node_types = {
            (self.new if name == self.old else name): ntype
            for name, ntype in model.node_types.items()
        }

        model.line_segments = [
            self._rename_edge(edge) for edge in model.line_segments
        ]
        model.blind_edges = [
            self._rename_edge(edge) for edge in model.blind_edges
        ]

        self._start_node_was_old = model.start_node == self.old
        if self._start_node_was_old:
            model.start_node = self.new

    def revert(self, model: FieldModel) -> None:
        """Reverse the rename, restoring the original name in every container.

        :param model: Target model.
        """
        model.nodes = {
            (self.old if name == self.new else name): xy
            for name, xy in model.nodes.items()
        }
        model.node_types = {
            (self.old if name == self.new else name): ntype
            for name, ntype in model.node_types.items()
        }
        model.line_segments = [
            self._unrename_edge(edge) for edge in model.line_segments
        ]
        model.blind_edges = [
            self._unrename_edge(edge) for edge in model.blind_edges
        ]
        if self._start_node_was_old:
            model.start_node = self.old

    # -- internal helpers ------------------------------------------------

    def _rename_edge(self, edge: tuple[str, str]) -> tuple[str, str]:
        """Return *edge* with ``self.old`` replaced by ``self.new``.

        :param edge: Original edge tuple.
        :return: Edge with the rename applied to either or both endpoints.
        """
        a, b = edge
        return (
            self.new if a == self.old else a,
            self.new if b == self.old else b,
        )

    def _unrename_edge(self, edge: tuple[str, str]) -> tuple[str, str]:
        """Return *edge* with ``self.new`` replaced by ``self.old``.

        :param edge: Edge previously transformed by :meth:`_rename_edge`.
        :return: Edge with the rename reversed.
        """
        a, b = edge
        return (
            self.old if a == self.new else a,
            self.old if b == self.new else b,
        )


@dataclass
class DeleteNode:
    """Delete a node from the model.

    Refuses if any entry in ``model.line_segments`` or ``model.blind_edges``
    still references the node -- the user must delete those edges first.

    .. note::
       This command only checks the in-memory edge lists.  The rich ``EDGES``
       dictionary in ``field_data.py`` (per-edge metadata) is NOT visible to
       the :class:`FieldModel` and therefore NOT checked here.  T12's Save
       handler is responsible for peeking at the imported ``field_data``
       module before allowing the deletion to be flushed to disk.

    :ivar name: Name of the node to delete.
    """

    name: str

    _deleted_xy: tuple[float, float] | None = field(
        default=None, init=False, repr=False
    )
    _deleted_type: str | None = field(default=None, init=False, repr=False)

    def apply(self, model: FieldModel) -> None:
        """Remove ``self.name`` from ``model.nodes`` and ``model.node_types``.

        :param model: Target model.
        :raises KeyError: If ``self.name`` is absent from ``model.nodes``.
        :raises ValueError: If any line segment or blind edge still references
            the node.  The message names the offending list and index.
        """
        if self.name not in model.nodes:
            raise KeyError(
                f"DeleteNode: no node named {self.name!r} in model.nodes"
            )

        for index, edge in enumerate(model.line_segments):
            if self.name in edge:
                raise ValueError(
                    f"DeleteNode: cannot delete {self.name!r}; "
                    f"line_segments[{index}] = {edge!r} still references it"
                )
        for index, edge in enumerate(model.blind_edges):
            if self.name in edge:
                raise ValueError(
                    f"DeleteNode: cannot delete {self.name!r}; "
                    f"blind_edges[{index}] = {edge!r} still references it"
                )

        self._deleted_xy = model.nodes[self.name]
        # ``node_types`` may be missing in a malformed model; default to a
        # safe value for revert so we never re-introduce a half-deleted node.
        self._deleted_type = model.node_types.get(self.name)
        del model.nodes[self.name]
        model.node_types.pop(self.name, None)

    def revert(self, model: FieldModel) -> None:
        """Re-insert the node that :meth:`apply` removed.

        :param model: Target model.
        """
        assert self._deleted_xy is not None, "DeleteNode.revert before apply"
        model.nodes[self.name] = self._deleted_xy
        if self._deleted_type is not None:
            model.node_types[self.name] = self._deleted_type


@dataclass
class SetNodeType:
    """Change a node's type between ``"line"`` and ``"blind"``.

    :ivar name: Name of the node whose type is being changed.
    :ivar new_type: New type; must be ``"line"`` or ``"blind"``.
    """

    name: str
    new_type: str

    _original_type: str | None = field(default=None, init=False, repr=False)

    def apply(self, model: FieldModel) -> None:
        """Set ``model.node_types[self.name]`` to ``self.new_type``.

        :param model: Target model.
        :raises KeyError: If ``self.name`` is not in ``model.node_types``.
        :raises ValueError: If ``self.new_type`` is not ``"line"`` or
            ``"blind"``.
        """
        if self.new_type not in _VALID_NODE_TYPES:
            raise ValueError(
                f"SetNodeType: new_type must be one of "
                f"{sorted(_VALID_NODE_TYPES)!r}, got {self.new_type!r}"
            )
        if self.name not in model.node_types:
            raise KeyError(
                f"SetNodeType: no node named {self.name!r} in model.node_types"
            )
        self._original_type = model.node_types[self.name]
        model.node_types[self.name] = self.new_type

    def revert(self, model: FieldModel) -> None:
        """Restore the node's prior type.

        :param model: Target model.
        """
        assert self._original_type is not None, (
            "SetNodeType.revert before apply"
        )
        model.node_types[self.name] = self._original_type


# ---------------------------------------------------------------------------
# Edge commands
# ---------------------------------------------------------------------------


@dataclass
class AddLineEdge:
    """Append a line-following edge between two nodes.

    Refuses if any tuple equivalent to ``(a, b)`` (either ordering) already
    exists in ``model.line_segments`` OR ``model.blind_edges`` -- the graph
    is undirected and the two lists are mutually exclusive.

    :ivar a: First endpoint node name.
    :ivar b: Second endpoint node name.
    """

    a: str
    b: str

    def apply(self, model: FieldModel) -> None:
        """Append ``(self.a, self.b)`` to ``model.line_segments``.

        :param model: Target model.
        :raises ValueError: If the same undirected edge already exists in
            either ``model.line_segments`` or ``model.blind_edges``.
        """
        target = (self.a, self.b)
        if _find_edge_index(model.line_segments, target) >= 0:
            raise ValueError(
                f"AddLineEdge: edge {target!r} already exists in line_segments"
            )
        if _find_edge_index(model.blind_edges, target) >= 0:
            raise ValueError(
                f"AddLineEdge: edge {target!r} already exists in blind_edges"
            )
        model.line_segments.append(target)

    def revert(self, model: FieldModel) -> None:
        """Remove the edge that :meth:`apply` appended.

        :param model: Target model.
        """
        # The edge was appended at the end by ``apply``; pop from the end if
        # still there.  Defensive search guards against an external mutation
        # having shuffled the list between apply and revert.
        target = (self.a, self.b)
        index = _find_edge_index(model.line_segments, target)
        if index >= 0:
            del model.line_segments[index]


@dataclass
class AddBlindEdge:
    """Append a blind / dead-reckoning edge between two nodes.

    Symmetric counterpart to :class:`AddLineEdge`; same duplicate-detection
    rules apply across both edge lists.

    :ivar a: First endpoint node name.
    :ivar b: Second endpoint node name.
    """

    a: str
    b: str

    def apply(self, model: FieldModel) -> None:
        """Append ``(self.a, self.b)`` to ``model.blind_edges``.

        :param model: Target model.
        :raises ValueError: If the same undirected edge already exists in
            either ``model.line_segments`` or ``model.blind_edges``.
        """
        target = (self.a, self.b)
        if _find_edge_index(model.blind_edges, target) >= 0:
            raise ValueError(
                f"AddBlindEdge: edge {target!r} already exists in blind_edges"
            )
        if _find_edge_index(model.line_segments, target) >= 0:
            raise ValueError(
                f"AddBlindEdge: edge {target!r} already exists in line_segments"
            )
        model.blind_edges.append(target)

    def revert(self, model: FieldModel) -> None:
        """Remove the edge that :meth:`apply` appended.

        :param model: Target model.
        """
        target = (self.a, self.b)
        index = _find_edge_index(model.blind_edges, target)
        if index >= 0:
            del model.blind_edges[index]


@dataclass
class AddEdgesBatch:
    """Append multiple edges atomically.  One undo entry covers the whole batch.

    All edges are validated before any mutation takes place.  If any edge
    fails validation the model is left completely untouched (all-or-nothing).
    An empty batch is a legal no-op.

    :ivar edges: Sequence of ``(node_a, node_b, kind)`` triples where *kind*
        is either ``"line"`` or ``"blind"``.
    """

    edges: list[tuple[str, str, str]]

    # Populated during apply(); each entry is ("line"|"blind", appended_index).
    # Stored in input order; revert walks this list in reverse.
    _applied_indices: list[tuple[str, int]] | None = field(
        default=None, init=False, repr=False
    )

    def apply(self, model: FieldModel) -> None:
        """Validate all edges, then append them atomically to *model*.

        **First pass (validation):** Each ``(a, b, kind)`` entry is checked in
        input order.  The check fails immediately (with a precise message) if:

        * *kind* is not ``"line"`` or ``"blind"``
        * Either endpoint is not in ``model.nodes``
        * The undirected pair ``{a, b}`` already exists in
          ``model.line_segments`` or ``model.blind_edges``
        * The same undirected pair appears more than once within
          ``self.edges`` itself

        No model mutation occurs if any check fails.

        **Second pass (append):** Each edge is appended to the matching list
        and its ``("line"|"blind", new_index)`` is recorded in
        ``_applied_indices``.

        :param model: Target model.
        :raises ValueError: On the first validation failure; the model is
            untouched.
        """
        # ── Pass 1: validate, collect no mutations ───────────────────────────
        # Canonical key for an undirected pair: frozenset so (a,b)==(b,a).
        seen_in_batch: set[frozenset[str]] = set()

        for entry in self.edges:
            a, b, kind = entry

            if kind not in _VALID_EDGE_KINDS:
                raise ValueError(
                    f"AddEdgesBatch: kind must be one of "
                    f"{sorted(_VALID_EDGE_KINDS)!r}, got {kind!r} "
                    f"in edge {entry!r}"
                )

            if a not in model.nodes:
                raise ValueError(
                    f"AddEdgesBatch: unknown node {a!r} in edge {entry!r}"
                )
            if b not in model.nodes:
                raise ValueError(
                    f"AddEdgesBatch: unknown node {b!r} in edge {entry!r}"
                )

            pair = frozenset({a, b})
            if pair in seen_in_batch:
                raise ValueError(
                    f"AddEdgesBatch: duplicate pair within batch: {entry!r}"
                )

            line_idx = _find_edge_index(model.line_segments, (a, b))
            if line_idx >= 0:
                raise ValueError(
                    f"AddEdgesBatch: edge {entry!r} conflicts with existing "
                    f"line_segments[{line_idx}]"
                )

            blind_idx = _find_edge_index(model.blind_edges, (a, b))
            if blind_idx >= 0:
                raise ValueError(
                    f"AddEdgesBatch: edge {entry!r} conflicts with existing "
                    f"blind_edges[{blind_idx}]"
                )

            seen_in_batch.add(pair)

        # ── Pass 2: append ────────────────────────────────────────────────────
        applied: list[tuple[str, int]] = []
        for a, b, kind in self.edges:
            if kind == "line":
                new_index = len(model.line_segments)
                model.line_segments.append((a, b))
                applied.append(("line", new_index))
            else:  # kind == "blind"
                new_index = len(model.blind_edges)
                model.blind_edges.append((a, b))
                applied.append(("blind", new_index))

        self._applied_indices = applied

    def revert(self, model: FieldModel) -> None:
        """Remove every edge that :meth:`apply` appended, in reverse order.

        Walking in reverse ensures that each deletion is always removing from
        the tail of its list (since apply only ever appended), so no earlier
        recorded index is disturbed by a later deletion.

        :param model: Target model.
        """
        assert self._applied_indices is not None, (
            "AddEdgesBatch.revert called before a successful apply"
        )
        for list_name, idx in reversed(self._applied_indices):
            if list_name == "line":
                del model.line_segments[idx]
            else:
                del model.blind_edges[idx]


@dataclass
class DeleteEdge:
    """Remove an edge from whichever list (line or blind) it lives in.

    Edges are undirected, so the command matches either ordering.

    :ivar a: First endpoint node name.
    :ivar b: Second endpoint node name.
    """

    a: str
    b: str

    # ``"line"`` or ``"blind"`` -- which list the edge was removed from.
    _origin_list: str | None = field(default=None, init=False, repr=False)
    _origin_index: int | None = field(default=None, init=False, repr=False)
    _origin_value: tuple[str, str] | None = field(
        default=None, init=False, repr=False
    )

    def apply(self, model: FieldModel) -> None:
        """Locate and remove the edge from ``line_segments`` or ``blind_edges``.

        :param model: Target model.
        :raises ValueError: If the edge is not found in either list.
        """
        target = (self.a, self.b)
        line_index = _find_edge_index(model.line_segments, target)
        if line_index >= 0:
            self._origin_list = "line"
            self._origin_index = line_index
            self._origin_value = model.line_segments[line_index]
            del model.line_segments[line_index]
            return

        blind_index = _find_edge_index(model.blind_edges, target)
        if blind_index >= 0:
            self._origin_list = "blind"
            self._origin_index = blind_index
            self._origin_value = model.blind_edges[blind_index]
            del model.blind_edges[blind_index]
            return

        raise ValueError(
            f"DeleteEdge: edge ({self.a!r}, {self.b!r}) not found in "
            "line_segments or blind_edges"
        )

    def revert(self, model: FieldModel) -> None:
        """Re-insert the edge at its original index in its original list.

        :param model: Target model.
        """
        assert self._origin_list is not None, "DeleteEdge.revert before apply"
        assert self._origin_index is not None
        assert self._origin_value is not None
        if self._origin_list == "line":
            model.line_segments.insert(self._origin_index, self._origin_value)
        else:
            model.blind_edges.insert(self._origin_index, self._origin_value)


# ---------------------------------------------------------------------------
# Obstacle commands
# ---------------------------------------------------------------------------


@dataclass
class AddObstacle:
    """Append an axis-aligned rectangular obstacle to the model.

    :ivar xmin: West edge in centimetres.
    :ivar ymin: South edge in centimetres.
    :ivar xmax: East edge in centimetres.  Must be strictly greater than
        ``xmin``.
    :ivar ymax: North edge in centimetres.  Must be strictly greater than
        ``ymin``.
    :ivar kind: Short tag (e.g. ``"wall"``, ``"box"``); free-form string used
        by the visualizer.
    """

    xmin: float
    ymin: float
    xmax: float
    ymax: float
    kind: str

    def apply(self, model: FieldModel) -> None:
        """Append the obstacle tuple to ``model.obstacles``.

        :param model: Target model.
        :raises ValueError: If the rectangle is degenerate
            (``xmax <= xmin`` or ``ymax <= ymin``).
        """
        if self.xmax <= self.xmin or self.ymax <= self.ymin:
            raise ValueError(
                f"AddObstacle: degenerate rectangle "
                f"({self.xmin}, {self.ymin}, {self.xmax}, {self.ymax})"
            )
        model.obstacles.append(
            (self.xmin, self.ymin, self.xmax, self.ymax, self.kind)
        )

    def revert(self, model: FieldModel) -> None:
        """Remove the obstacle that :meth:`apply` appended.

        :param model: Target model.
        """
        target = (self.xmin, self.ymin, self.xmax, self.ymax, self.kind)
        # Walk from the end since ``apply`` appends; this also tolerates the
        # case where other commands appended after us and then reverted, so
        # the original entry is no longer at the very tail.
        for index in range(len(model.obstacles) - 1, -1, -1):
            if model.obstacles[index] == target:
                del model.obstacles[index]
                return


@dataclass
class DeleteObstacle:
    """Remove an obstacle from ``model.obstacles`` by index.

    :ivar index: Position in ``model.obstacles`` to remove.
    """

    index: int

    _removed: tuple[float, float, float, float, str] | None = field(
        default=None, init=False, repr=False
    )

    def apply(self, model: FieldModel) -> None:
        """Remove and capture ``model.obstacles[self.index]``.

        :param model: Target model.
        :raises IndexError: If ``self.index`` is not a valid index into
            ``model.obstacles``.
        """
        if self.index < 0 or self.index >= len(model.obstacles):
            raise IndexError(
                f"DeleteObstacle: index {self.index} out of range "
                f"(len(obstacles)={len(model.obstacles)})"
            )
        self._removed = model.obstacles[self.index]
        del model.obstacles[self.index]

    def revert(self, model: FieldModel) -> None:
        """Re-insert the removed obstacle at its original index.

        :param model: Target model.
        """
        assert self._removed is not None, "DeleteObstacle.revert before apply"
        model.obstacles.insert(self.index, self._removed)


# ---------------------------------------------------------------------------
# Start-pose commands
# ---------------------------------------------------------------------------


@dataclass
class SetStart:
    """Change ``model.start_node`` to a different existing node.

    :ivar new_node: Name of the node to use as the new start position.  Must
        exist in ``model.nodes``.
    """

    new_node: str

    _original: str | None = field(default=None, init=False, repr=False)

    def apply(self, model: FieldModel) -> None:
        """Set ``model.start_node = self.new_node``.

        :param model: Target model.
        :raises ValueError: If ``self.new_node`` is not in ``model.nodes``.
        """
        if self.new_node not in model.nodes:
            raise ValueError(
                f"SetStart: node {self.new_node!r} is not in model.nodes"
            )
        self._original = model.start_node
        model.start_node = self.new_node

    def revert(self, model: FieldModel) -> None:
        """Restore the prior ``start_node`` value.

        :param model: Target model.
        """
        assert self._original is not None, "SetStart.revert before apply"
        model.start_node = self._original


@dataclass
class SetStartHeading:
    """Change ``model.start_heading_deg``.

    :ivar new_heading_deg: New heading in degrees, in the half-open range
        ``[0, 360)`` (CCW from +x, matching the ``field_data`` convention).
    """

    new_heading_deg: float

    _original: float | None = field(default=None, init=False, repr=False)

    def apply(self, model: FieldModel) -> None:
        """Set ``model.start_heading_deg = self.new_heading_deg``.

        :param model: Target model.
        :raises ValueError: If ``self.new_heading_deg`` is outside
            ``[0, 360)``.
        """
        if not (
            _HEADING_MIN_DEG
            <= self.new_heading_deg
            < _HEADING_MAX_DEG_EXCLUSIVE
        ):
            raise ValueError(
                f"SetStartHeading: heading must be in "
                f"[{_HEADING_MIN_DEG}, {_HEADING_MAX_DEG_EXCLUSIVE}), "
                f"got {self.new_heading_deg}"
            )
        self._original = model.start_heading_deg
        model.start_heading_deg = self.new_heading_deg

    def revert(self, model: FieldModel) -> None:
        """Restore the prior heading.

        :param model: Target model.
        """
        assert self._original is not None, (
            "SetStartHeading.revert before apply"
        )
        model.start_heading_deg = self._original
