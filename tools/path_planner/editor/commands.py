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

from tools.path_planner.editor import projection_edges
from tools.path_planner.editor.model import FieldModel

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

#: All model-recognized values for ``FieldModel.node_types`` entries.  Note
#: that ``"reset"`` is special: a reset-typed node is invalid at Save unless it
#: also has a matching ``reset_nodes`` spec, so the plain type-setters
#: (:class:`AddNode`, :class:`SetNodeType`) refuse it and route reset creation
#: through :class:`SetResetSpec` / :class:`ClearResetSpec` instead.
_VALID_NODE_TYPES: frozenset[str] = frozenset({"line", "blind", "reset"})

#: Plain (non-reset) node types the type-setters and :class:`ClearResetSpec`
#: are allowed to assign directly.
_PLAIN_NODE_TYPES: frozenset[str] = frozenset({"line", "blind"})

#: Allowed ``kind`` values for edge-batch entries.
_VALID_EDGE_KINDS: frozenset[str] = frozenset({"line", "blind"})

#: Allowed ``axis`` values for a reset-node spec (the wall is parallel to one
#: field axis); mirrors ``validator._ALLOWED_RESET_AXES``.
_VALID_RESET_AXES: frozenset[str] = frozenset({"x", "y"})

#: Heading bounds, in degrees, CCW from +x (matches ``field_data`` convention).
_HEADING_MIN_DEG: float = 0.0
_HEADING_MAX_DEG_EXCLUSIVE: float = 360.0

#: Recognized global-knob attribute names :class:`SetGlobalKnob` may set, in
#: canonical display order.  Each maps to a same-named ``FieldModel`` field.
_VALID_GLOBAL_KNOBS: frozenset[str] = frozenset(
    {
        "blind_decay_enabled",
        "blind_decay_k",
        "blind_decay_exponent",
        "blind_decay_d_ref",
        "robot_radius_cm",
    }
)

#: Lower bound (inclusive) for ``blind_decay_k`` -- ``k == 0`` disables the
#: penalty, negative ``k`` is nonsensical.
_BLIND_DECAY_K_MIN: float = 0.0

#: Lower bound (inclusive) for ``blind_decay_exponent`` -- a sub-linear
#: exponent would invert the intended monotonic penalty shaping.
_BLIND_DECAY_EXPONENT_MIN: float = 1.0

#: Lower bound (exclusive) for the strictly-positive knobs ``blind_decay_d_ref``
#: (the decay normalizer in cm) and ``robot_radius_cm`` (the inflation radius).
_POSITIVE_KNOB_MIN_EXCLUSIVE: float = 0.0


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
    :ivar node_type: Either ``"line"`` or ``"blind"``.  A ``"reset"`` node may
        NOT be created here -- it requires a matching ``reset_nodes`` spec, so
        use :class:`SetResetSpec` instead.
    """

    name: str
    xy: tuple[float, float]
    node_type: str

    def apply(self, model: FieldModel) -> None:
        """Insert ``self.name`` into ``model.nodes`` and ``model.node_types``.

        :param model: Target model.
        :raises ValueError: If *node_type* is not a recognized type, if
            *node_type* is ``"reset"`` (a reset node must be created via
            :class:`SetResetSpec` so its spec is set atomically), or if *name*
            already exists in ``model.nodes`` or ``model.node_types``.
        """
        if self.node_type not in _VALID_NODE_TYPES:
            raise ValueError(
                f"AddNode: node_type must be one of "
                f"{sorted(_VALID_NODE_TYPES)!r}, got {self.node_type!r}"
            )
        if self.node_type == "reset":
            raise ValueError(
                "AddNode: cannot create a 'reset' node directly; a reset node "
                "needs a matching reset_nodes spec -- use SetResetSpec instead"
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
    still references the node, or if the node is the current
    ``model.start_node`` -- the user must delete those edges (or re-point the
    start node) first.

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
            the node, or if the node is the current ``model.start_node``.  The
            message names the offending list and index (or the start-node
            conflict).
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

        # Guard the start node last, so an otherwise-blocking edge reference
        # still reports its own (more actionable) message first. Deleting the
        # start node would leave ``model.start_node`` dangling, which
        # validate() rejects at the next Save and blocks every later Save until
        # a new start is picked -- refuse it here instead.
        if self.name == model.start_node:
            raise ValueError(
                f"DeleteNode: cannot delete {self.name!r}; it is the current "
                "start node -- pick a different start node first"
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
    """Change a node's type between the plain types ``"line"`` and ``"blind"``.

    This command deliberately cannot touch the ``"reset"`` type, in either
    direction: converting a node TO reset needs a paired ``reset_nodes`` spec
    (use :class:`SetResetSpec`), and converting a reset node back to a plain
    type needs the spec removed (use :class:`ClearResetSpec`).  Routing the
    reset type exclusively through those two commands makes the
    type-and-spec invariant the validator enforces impossible to break through
    the plain path.

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
        :raises ValueError: If ``self.new_type`` is not a recognized type, if
            ``self.new_type`` is ``"reset"`` (convert to reset via
            :class:`SetResetSpec`), or if the node is CURRENTLY a reset node
            (change a reset node's type via :class:`ClearResetSpec`).
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
        if self.new_type == "reset":
            raise ValueError(
                "SetNodeType: cannot set a node to 'reset' here; a reset node "
                "needs a matching reset_nodes spec -- use SetResetSpec instead"
            )
        if model.node_types[self.name] == "reset":
            raise ValueError(
                f"SetNodeType: node {self.name!r} is currently a reset node; "
                "change a reset node's type via ClearResetSpec instead"
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
# Property-inspector commands
#
# These commands back the property panel (T9): pivot turns, reset-node specs,
# and the global decay / robot-radius knobs.  They follow the same apply /
# revert + captured-inverse contract as the node and edge commands above, and
# raise their documented exception BEFORE mutating so a rejected apply is a
# clean no-op for ``History``.
# ---------------------------------------------------------------------------


@dataclass
class AddPivotTurn:
    """Append a single-wheel pivot turn ``(u, v, w)`` to ``model.pivot_turns``.

    A pivot turn directs the robot to arrive at ``v`` from ``u``, pivot in
    place at ``v``, and leave toward ``w``.  All three endpoints must be
    existing nodes, and the exact triple must not already be present.

    :ivar u: Arrive-from node name.
    :ivar v: Pivot node name.
    :ivar w: Leave-to node name.
    """

    u: str
    v: str
    w: str

    def apply(self, model: FieldModel) -> None:
        """Append ``(self.u, self.v, self.w)`` to ``model.pivot_turns``.

        :param model: Target model.
        :raises ValueError: If any of ``u`` / ``v`` / ``w`` is not in
            ``model.nodes``, or if the exact triple already exists in
            ``model.pivot_turns``.
        """
        for role, name in (("u", self.u), ("v", self.v), ("w", self.w)):
            if name not in model.nodes:
                raise ValueError(
                    f"AddPivotTurn: unknown node {name!r} ({role}) -- all of "
                    "u, v, w must exist in model.nodes"
                )
        turn = (self.u, self.v, self.w)
        if turn in model.pivot_turns:
            raise ValueError(
                f"AddPivotTurn: pivot turn {turn!r} already exists"
            )
        model.pivot_turns.append(turn)

    def revert(self, model: FieldModel) -> None:
        """Remove the pivot turn that :meth:`apply` appended.

        :param model: Target model.
        """
        # ``apply`` appends at the end; search by value so an external mutation
        # that shuffled the list between apply and revert is still handled.
        turn = (self.u, self.v, self.w)
        for index in range(len(model.pivot_turns) - 1, -1, -1):
            if model.pivot_turns[index] == turn:
                del model.pivot_turns[index]
                return


@dataclass
class DeletePivotTurn:
    """Remove a pivot turn ``(u, v, w)`` from ``model.pivot_turns``.

    :ivar turn: The ``(u, v, w)`` triple to remove.  Must be present in
        ``model.pivot_turns``.
    """

    turn: tuple[str, str, str]

    _removed_index: int | None = field(default=None, init=False, repr=False)
    _removed_value: tuple[str, str, str] | None = field(
        default=None, init=False, repr=False
    )

    def apply(self, model: FieldModel) -> None:
        """Locate and remove ``self.turn`` from ``model.pivot_turns``.

        :param model: Target model.
        :raises ValueError: If ``self.turn`` is not present in
            ``model.pivot_turns``.
        """
        for index, existing in enumerate(model.pivot_turns):
            if existing == self.turn:
                self._removed_index = index
                self._removed_value = existing
                del model.pivot_turns[index]
                return
        raise ValueError(
            f"DeletePivotTurn: pivot turn {self.turn!r} not found in "
            "model.pivot_turns"
        )

    def revert(self, model: FieldModel) -> None:
        """Re-insert the removed pivot turn at its original index.

        :param model: Target model.
        """
        assert self._removed_index is not None, (
            "DeletePivotTurn.revert before apply"
        )
        assert self._removed_value is not None
        model.pivot_turns.insert(self._removed_index, self._removed_value)


@dataclass
class SetResetSpec:
    """Convert a node to a wall-zero reset node, or edit its existing spec.

    This command OWNS the atomic type-and-spec change: it sets
    ``model.node_types[name] = "reset"`` and writes the paired
    ``model.reset_nodes[name]`` entry together, so the type-and-spec invariant
    the validator enforces (every reset-typed node has a spec and vice-versa)
    is never transiently violated.  It is used both to first-convert a plain
    node to a reset node and to re-author an existing reset node's spec.

    :ivar name: Name of the node to make (or keep) a reset node.  Must exist in
        ``model.nodes``.
    :ivar axis: Wall axis, either ``"x"`` or ``"y"`` (the wall is parallel to
        one field axis).
    :ivar wall_coord: Wall coordinate in centimetres along ``axis``.
    :ivar facing: Parked heading in degrees, CCW from +x.
    """

    name: str
    axis: str
    wall_coord: float
    facing: float

    # Captured at apply time so revert can restore both halves of the change.
    # ``_had_type`` distinguishes "node_types had no entry" (revert via pop)
    # from "node_types entry was None"; ``_had_spec`` does the same for
    # reset_nodes (revert via key removal vs restore).
    _original_type: str | None = field(default=None, init=False, repr=False)
    _had_type: bool = field(default=False, init=False, repr=False)
    _original_spec: dict | None = field(default=None, init=False, repr=False)
    _had_spec: bool = field(default=False, init=False, repr=False)

    def apply(self, model: FieldModel) -> None:
        """Set the node's type to ``"reset"`` and write its reset spec.

        :param model: Target model.
        :raises ValueError: If ``self.name`` is not in ``model.nodes`` or if
            ``self.axis`` is not ``"x"`` / ``"y"``.
        """
        if self.name not in model.nodes:
            raise ValueError(
                f"SetResetSpec: node {self.name!r} is not in model.nodes"
            )
        if self.axis not in _VALID_RESET_AXES:
            raise ValueError(
                f"SetResetSpec: axis must be one of "
                f"{sorted(_VALID_RESET_AXES)!r}, got {self.axis!r}"
            )
        self._had_type = self.name in model.node_types
        self._original_type = model.node_types.get(self.name)
        self._had_spec = self.name in model.reset_nodes
        # Deep-copy the original spec so a later in-place edit of the live dict
        # cannot mutate the captured inverse.
        original_spec = model.reset_nodes.get(self.name)
        self._original_spec = (
            dict(original_spec) if original_spec is not None else None
        )
        model.node_types[self.name] = "reset"
        model.reset_nodes[self.name] = {
            "axis": self.axis,
            "wall_coord": float(self.wall_coord),
            "facing": float(self.facing),
        }

    def revert(self, model: FieldModel) -> None:
        """Restore the node's prior type and reset-spec state.

        :param model: Target model.
        """
        if self._had_type:
            # ``_original_type`` may legitimately be None if node_types stored
            # an explicit None; restore it verbatim.
            model.node_types[self.name] = self._original_type  # type: ignore[assignment]
        else:
            model.node_types.pop(self.name, None)

        if self._had_spec:
            assert self._original_spec is not None
            model.reset_nodes[self.name] = dict(self._original_spec)
        else:
            model.reset_nodes.pop(self.name, None)

    @classmethod
    def autofilled(
        cls,
        name: str,
        node_xy: tuple[float, float],
        axis: str,
        field_mm: tuple[float, float],
    ) -> "SetResetSpec":
        """Build a :class:`SetResetSpec` with auto-suggested wall + facing.

        Seeds the ``wall_coord`` / ``facing`` pair from
        :func:`projection_edges.reset_autofill`, which picks the nearer
        qualifying field wall (parallel to *axis*) and the heading the chassis
        faces while standing off it. The app (T8) supplies
        ``node_xy=model.nodes[name]`` and the field dimensions
        (``model.field_mm``); keeping the inputs pure (no ``FieldModel``) makes
        this classmethod directly unit-testable.

        :param name: Name of the node to make (or keep) a reset node. Not
            validated here -- :meth:`apply` rejects an unknown node at command
            time.
        :param node_xy: The reset node position ``(x_cm, y_cm)``, used to choose
            the nearer wall.
        :param axis: Wall axis, either ``"x"`` or ``"y"``.
        :param field_mm: Outer field size ``(width_mm, height_mm)`` in
            millimetres (the ``FieldModel.field_mm`` value).
        :return: A :class:`SetResetSpec` whose ``axis`` is *axis* and whose
            ``wall_coord`` / ``facing`` are the auto-filled suggestion.
        :raises ValueError: If *axis* is not ``"x"`` or ``"y"`` (propagated from
            :func:`projection_edges.reset_autofill`).
        """
        wall_coord, facing = projection_edges.reset_autofill(
            node_xy, axis, field_mm
        )
        return cls(
            name=name, axis=axis, wall_coord=wall_coord, facing=facing
        )


@dataclass
class ClearResetSpec:
    """Revert a reset node back to a plain ``"line"`` / ``"blind"`` node.

    Removes the ``model.reset_nodes`` entry and re-types the node, the inverse
    of :class:`SetResetSpec`.  Refuses any node that is not currently a fully
    formed reset node (reset type AND a spec entry), so the type-and-spec
    invariant cannot be broken from this side either.

    :ivar name: Name of the reset node to clear.  Must currently be a reset
        node.
    :ivar new_type: Plain type to assign after clearing; ``"line"`` (default)
        or ``"blind"``.
    """

    name: str
    new_type: str = "line"

    _original_type: str | None = field(default=None, init=False, repr=False)
    _removed_spec: dict | None = field(default=None, init=False, repr=False)

    def apply(self, model: FieldModel) -> None:
        """Remove the reset spec and set the node to a plain type.

        :param model: Target model.
        :raises ValueError: If ``self.name`` is not in ``model.nodes``, if
            ``self.new_type`` is not ``"line"`` / ``"blind"``, or if the node
            is not currently a fully formed reset node (reset type and a
            matching ``reset_nodes`` entry).
        """
        if self.name not in model.nodes:
            raise ValueError(
                f"ClearResetSpec: node {self.name!r} is not in model.nodes"
            )
        if self.new_type not in _PLAIN_NODE_TYPES:
            raise ValueError(
                f"ClearResetSpec: new_type must be one of "
                f"{sorted(_PLAIN_NODE_TYPES)!r}, got {self.new_type!r}"
            )
        if (
            model.node_types.get(self.name) != "reset"
            or self.name not in model.reset_nodes
        ):
            raise ValueError(
                f"ClearResetSpec: node {self.name!r} is not a reset node "
                "(needs node_types[name] == 'reset' and a reset_nodes entry)"
            )
        self._original_type = model.node_types[self.name]
        # Deep-copy so the captured inverse is immune to later live mutation.
        self._removed_spec = dict(model.reset_nodes[self.name])
        del model.reset_nodes[self.name]
        model.node_types[self.name] = self.new_type

    def revert(self, model: FieldModel) -> None:
        """Restore the original reset type and re-insert the removed spec.

        :param model: Target model.
        """
        assert self._original_type is not None, (
            "ClearResetSpec.revert before apply"
        )
        assert self._removed_spec is not None
        model.node_types[self.name] = self._original_type
        model.reset_nodes[self.name] = dict(self._removed_spec)


@dataclass
class SetGlobalKnob:
    """Set one global model knob to a validated value (a keyed setter).

    Backs the property inspector's global-knob controls (decay enable / ``k`` /
    exponent / ``d_ref`` and the robot inflation radius).  Each knob name maps
    to a same-named ``FieldModel`` attribute; the value is coerced and
    range-checked per-knob BEFORE the attribute is touched, so a rejected value
    leaves the model untouched.

    :ivar knob: Attribute name to set; one of ``_VALID_GLOBAL_KNOBS``.
    :ivar value: New value.  Type and bounds are enforced per-knob (see
        :meth:`apply`).
    """

    knob: str
    value: object

    _original_value: object = field(default=None, init=False, repr=False)
    _had_value: bool = field(default=False, init=False, repr=False)

    def apply(self, model: FieldModel) -> None:
        """Validate ``self.value`` for ``self.knob`` and set the attribute.

        Bounds enforced per-knob, all checked before any mutation:

        * ``blind_decay_enabled`` -> coerced to ``bool``.
        * ``blind_decay_k`` -> ``float``, ``>= 0``.
        * ``blind_decay_exponent`` -> ``float``, ``>= 1``.
        * ``blind_decay_d_ref`` -> ``float``, ``> 0``.
        * ``robot_radius_cm`` -> ``float``, ``> 0``.

        :param model: Target model.
        :raises ValueError: If ``self.knob`` is not a recognized knob name, or
            if ``self.value`` is out of range / not coercible for that knob.
            The model is untouched in either case.
        """
        if self.knob not in _VALID_GLOBAL_KNOBS:
            raise ValueError(
                f"SetGlobalKnob: knob must be one of "
                f"{sorted(_VALID_GLOBAL_KNOBS)!r}, got {self.knob!r}"
            )

        coerced = self._coerce_and_validate()

        # Capture the inverse only after validation has fully passed, so a
        # rejected value never records phantom inverse state.
        self._had_value = hasattr(model, self.knob)
        self._original_value = getattr(model, self.knob, None)
        setattr(model, self.knob, coerced)

    def revert(self, model: FieldModel) -> None:
        """Restore the knob's prior value.

        :param model: Target model.
        """
        assert self._had_value, "SetGlobalKnob.revert before apply"
        setattr(model, self.knob, self._original_value)

    # -- internal helpers ------------------------------------------------

    def _coerce_and_validate(self) -> object:
        """Coerce and range-check ``self.value`` for ``self.knob``.

        :return: The coerced value to store on the model.
        :raises ValueError: If the value is not coercible or out of range.
        """
        if self.knob == "blind_decay_enabled":
            return bool(self.value)

        # Remaining knobs are all floats; coerce once, then bound per-knob.
        try:
            number = float(self.value)  # type: ignore[arg-type]
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f"SetGlobalKnob: {self.knob} requires a number, "
                f"got {self.value!r}"
            ) from exc

        if self.knob == "blind_decay_k":
            if number < _BLIND_DECAY_K_MIN:
                raise ValueError(
                    f"SetGlobalKnob: blind_decay_k must be "
                    f">= {_BLIND_DECAY_K_MIN}, got {number}"
                )
        elif self.knob == "blind_decay_exponent":
            if number < _BLIND_DECAY_EXPONENT_MIN:
                raise ValueError(
                    f"SetGlobalKnob: blind_decay_exponent must be "
                    f">= {_BLIND_DECAY_EXPONENT_MIN}, got {number}"
                )
        elif self.knob == "blind_decay_d_ref":
            if number <= _POSITIVE_KNOB_MIN_EXCLUSIVE:
                raise ValueError(
                    f"SetGlobalKnob: blind_decay_d_ref must be "
                    f"> {_POSITIVE_KNOB_MIN_EXCLUSIVE} cm, got {number}"
                )
        elif self.knob == "robot_radius_cm":
            if number <= _POSITIVE_KNOB_MIN_EXCLUSIVE:
                raise ValueError(
                    f"SetGlobalKnob: robot_radius_cm must be "
                    f"> {_POSITIVE_KNOB_MIN_EXCLUSIVE} cm, got {number}"
                )

        return number


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
# Projection-edge commands
#
# Projection edges are NOT part of ``FieldModel`` / the JSON schema: they live
# in ``field_data.py``'s ``EDGES`` list and are flushed at Save via LibCST
# (T8).  To make add / remove undoable through the SAME ``History`` as every
# other edit, the app owns a single :class:`ProjectionDelta` and the commands
# below mutate THAT delta -- never the model.  ``model`` is consulted only for
# read-only validation (node existence).  The delta's ``render()`` is the
# source of truth the canvas paints and the Save flow flushes.
# ---------------------------------------------------------------------------


@dataclass
class ProjectionDelta:
    """App-owned, undoable working set of projection edges.

    The app holds exactly one of these.  ``loaded`` is the baseline read from
    ``field_data.py``'s ``EDGES`` at Open (and re-seeded after Save);
    ``adds`` and ``removes`` are the pending session edits the
    :class:`AddProjectionEdge` / :class:`DeleteProjectionEdge` commands push
    and pop.  :meth:`render` folds the three into the deterministic set the
    canvas paints and the Save flow flushes.

    The delta is plain data: it owns no model reference and performs no
    validation.  Node-existence checks live in the commands (which have the
    model in hand); the delta only tracks membership.

    :ivar loaded: Baseline projection edges read from ``field_data.py`` EDGES,
        each ``(u, v, distance_cm, heading_deg)``.  Set by the app at load and
        re-seeded after Save.
    :ivar adds: Pending new projection edges, each
        ``(u, v, distance_cm, heading_deg)``.
    :ivar removes: Pending removals of *loaded* edges, each ``(u, v)``.
    """

    loaded: list[tuple[str, str, float, float]] = field(default_factory=list)
    adds: list[tuple[str, str, float, float]] = field(default_factory=list)
    removes: list[tuple[str, str]] = field(default_factory=list)

    def render(self) -> list[tuple[str, str, float, float]]:
        """Return the current projection-edge render set.

        Delegates to :func:`projection_edges.reconcile`, which yields the
        surviving ``loaded`` entries (source order) followed by the ``adds``
        (user order), with every ``(u, v)`` in ``removes`` suppressed and a
        same-pair add overriding the loaded geometry.

        :return: ``(u, v, distance_cm, heading_deg)`` tuples in render order.
        """
        return projection_edges.reconcile(self.loaded, self.adds, self.removes)

    def has(self, u: str, v: str) -> bool:
        """Return ``True`` if ``(u, v)`` is in the current :meth:`render` set.

        Matching is on the first two elements only (the ``(u, v)`` pair); the
        distance / heading payload is not compared.

        :param u: Source node name.
        :param v: Destination node name.
        :return: Whether an edge with this exact ``(u, v)`` pair currently
            renders.
        """
        return any(edge[0] == u and edge[1] == v for edge in self.render())

    def set_loaded(
        self, loaded: list[tuple[str, str, float, float]]
    ) -> None:
        """Replace the baseline ``loaded`` set (app load / post-Save re-seed).

        :param loaded: The new baseline projection edges, each
            ``(u, v, distance_cm, heading_deg)``.
        """
        self.loaded = loaded

    def clear_pending(self) -> None:
        """Empty both pending lists (``adds`` and ``removes``).

        Called by the app's Save flow once the pending edits have been flushed
        into the baseline, so the next session starts from a clean delta.
        """
        self.adds.clear()
        self.removes.clear()


@dataclass
class AddProjectionEdge:
    """Add a projection edge ``u -> v`` to an app-owned :class:`ProjectionDelta`.

    Mutates ``self.delta`` only -- the ``model`` is consulted purely for
    read-only validation (both endpoints must exist).  Re-adding a *loaded*
    edge the user had pending-deleted is handled by un-cancelling that removal
    rather than appending a duplicate add, so an add then undo restores the
    delta byte-identically.

    :ivar delta: The app's single projection-edge working set to mutate.
    :ivar u: Source node name; must exist in ``model.nodes``.
    :ivar v: Destination node name; must exist in ``model.nodes`` and differ
        from *u*.
    :ivar distance_cm: Edge length in centimetres.
    :ivar heading_deg: Edge heading in degrees, CCW from +x.
    """

    delta: ProjectionDelta
    u: str
    v: str
    distance_cm: float
    heading_deg: float

    # ``"appended"`` -> a new 4-tuple was pushed onto ``delta.adds``;
    # ``"uncanceled_remove"`` -> a pending ``(u, v)`` removal of a loaded edge
    # was dropped from ``delta.removes`` (re-adding what the user had deleted).
    _mode: str | None = field(default=None, init=False, repr=False)

    def apply(self, model: FieldModel) -> None:
        """Add the ``(u, v)`` projection edge to ``self.delta``.

        Validates against the model (read-only) before any mutation, so a
        rejected add is a clean no-op for ``History``.

        :param model: Target model, used only for node-existence validation.
        :raises ValueError: If *u* or *v* is not in ``model.nodes``, if
            ``u == v``, or if ``(u, v)`` already renders in ``self.delta``.
        """
        if self.u not in model.nodes:
            raise ValueError(
                f"AddProjectionEdge: unknown source node {self.u!r}"
            )
        if self.v not in model.nodes:
            raise ValueError(
                f"AddProjectionEdge: unknown destination node {self.v!r}"
            )
        if self.u == self.v:
            raise ValueError(
                f"AddProjectionEdge: u and v must differ (both {self.u!r}); "
                "a projection edge needs two distinct endpoints"
            )
        if self.delta.has(self.u, self.v):
            raise ValueError(
                f"AddProjectionEdge: edge ({self.u!r}, {self.v!r}) already "
                "exists in the projection delta"
            )

        # Re-adding a loaded edge the user had pending-deleted: drop the
        # removal instead of appending a (would-be-shadowed) duplicate add.
        pair = (self.u, self.v)
        if pair in self.delta.removes:
            self.delta.removes.remove(pair)
            self._mode = "uncanceled_remove"
        else:
            self.delta.adds.append(
                (self.u, self.v, self.distance_cm, self.heading_deg)
            )
            self._mode = "appended"

    def revert(self, model: FieldModel) -> None:
        """Undo the add, restoring ``self.delta`` byte-identically.

        :param model: Unused (the delta, not the model, was mutated); accepted
            to satisfy the :class:`Command` protocol.
        """
        assert self._mode is not None, (
            "AddProjectionEdge.revert before apply"
        )
        if self._mode == "appended":
            # Remove the first ``(u, v)``-matching add this command appended.
            for index, edge in enumerate(self.delta.adds):
                if edge[0] == self.u and edge[1] == self.v:
                    del self.delta.adds[index]
                    return
        else:  # self._mode == "uncanceled_remove"
            # Re-cancel: restore the removal this command had dropped.
            self.delta.removes.append((self.u, self.v))


@dataclass
class DeleteProjectionEdge:
    """Remove a projection edge ``u -> v`` from an app-owned
    :class:`ProjectionDelta`.

    Mutates ``self.delta`` only -- the ``model`` is consulted purely for
    read-only validation (the edge must currently render).  A pending *add* is
    dropped outright (and re-inserted at its original index on revert); a
    *loaded* edge is suppressed by appending its ``(u, v)`` to ``removes``.

    :ivar delta: The app's single projection-edge working set to mutate.
    :ivar u: Source node name of the edge to remove.
    :ivar v: Destination node name of the edge to remove.
    """

    delta: ProjectionDelta
    u: str
    v: str

    # ``"dropped_add"`` -> a pending add was removed from ``delta.adds``;
    # ``"added_remove"`` -> a loaded edge was suppressed via ``delta.removes``.
    _mode: str | None = field(default=None, init=False, repr=False)
    # For ``"dropped_add"``: the removed 4-tuple and the index it lived at, so
    # revert re-inserts it exactly where it was.
    _dropped_add: tuple[str, str, float, float] | None = field(
        default=None, init=False, repr=False
    )
    _dropped_index: int | None = field(default=None, init=False, repr=False)

    def apply(self, model: FieldModel) -> None:
        """Remove the ``(u, v)`` projection edge from ``self.delta``.

        Validates that the edge currently renders before any mutation, so a
        rejected delete is a clean no-op for ``History``.

        :param model: Unused for mutation; accepted to satisfy the
            :class:`Command` protocol (validation is against the delta, not the
            model).
        :raises ValueError: If ``(u, v)`` is not in ``self.delta.render()``.
        """
        if not self.delta.has(self.u, self.v):
            raise ValueError(
                f"DeleteProjectionEdge: edge ({self.u!r}, {self.v!r}) is not "
                "in the projection delta"
            )

        # A pending add for this pair is dropped outright; otherwise the edge
        # is a loaded one and is suppressed via ``removes``.
        for index, edge in enumerate(self.delta.adds):
            if edge[0] == self.u and edge[1] == self.v:
                self._dropped_add = edge
                self._dropped_index = index
                del self.delta.adds[index]
                self._mode = "dropped_add"
                return

        self.delta.removes.append((self.u, self.v))
        self._mode = "added_remove"

    def revert(self, model: FieldModel) -> None:
        """Undo the delete, restoring ``self.delta`` byte-identically.

        :param model: Unused (the delta, not the model, was mutated); accepted
            to satisfy the :class:`Command` protocol.
        """
        assert self._mode is not None, (
            "DeleteProjectionEdge.revert before apply"
        )
        if self._mode == "dropped_add":
            assert self._dropped_add is not None
            assert self._dropped_index is not None
            self.delta.adds.insert(self._dropped_index, self._dropped_add)
        else:  # self._mode == "added_remove"
            # Remove the ``(u, v)`` suppression this command appended.
            pair = (self.u, self.v)
            for index in range(len(self.delta.removes) - 1, -1, -1):
                if self.delta.removes[index] == pair:
                    del self.delta.removes[index]
                    return


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
