"""In-memory field model for the WRO 2026 live field editor.

``FieldModel`` is a plain Python dataclass that mirrors the ``field_data.json``
schema.  It is the single source of truth for the editor while the application
is running: every GUI mutation goes through a :class:`Command` that modifies a
``FieldModel`` instance, and the undo/redo stack saves and restores
:meth:`FieldModel.snapshot` copies.

No I/O is performed here.  Loading from disk and writing back are handled by
:mod:`tools.path_planner.editor.loader`.

Coordinate convention (inherited from ``field_data.py``):
    Origin at the field SW corner.  X is East, Y is North.  Distances in
    centimetres.  Headings in degrees, [0, 360), CCW from the +x axis.
"""

from __future__ import annotations

import copy
import types
from dataclasses import dataclass
from typing import Any, ClassVar


@dataclass
class FieldModel:
    """Mirror of the ``field_data.json`` schema held in memory while editing.

    :param field_mm: Outer field dimensions ``(width_mm, height_mm)``.
    :param map_png: Relative path (from ``tools/path_planner/``) to the
        background PNG used by the visualizer.
    :param robot_radius_cm: Inflation radius in centimetres applied to each
        obstacle before edge-intersection checks.
    :param cost_blind_multiplier: Dijkstra cost multiplier applied to blind
        edges so the planner prefers line-following paths.
    :param start_node: Name of the node where the robot begins.
    :param start_heading_deg: Initial robot heading in degrees, CCW from +x.
    :param nodes: Mapping of node name to ``(x_cm, y_cm)`` position.
    :param node_types: Mapping of node name to ``"line"`` or ``"blind"``.
    :param line_segments: Undirected line-following edges, each as
        ``(node_a, node_b)``.
    :param blind_edges: Undirected dead-reckoning edges, each as
        ``(node_a, node_b)``.
    :param obstacles: Axis-aligned rectangles as
        ``(x_min, y_min, x_max, y_max, kind)``.
    """

    # Ordered list of top-level JSON keys.  ``to_json_dict`` produces keys in
    # this exact order so the output file is deterministic across Python
    # versions and dict implementations.
    _JSON_KEY_ORDER: ClassVar[tuple[str, ...]] = (
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

    field_mm: tuple[float, float]
    map_png: str
    robot_radius_cm: float
    cost_blind_multiplier: float
    start_node: str
    start_heading_deg: float
    nodes: dict[str, tuple[float, float]]
    node_types: dict[str, str]
    line_segments: list[tuple[str, str]]
    blind_edges: list[tuple[str, str]]
    obstacles: list[tuple[float, float, float, float, str]]

    # ------------------------------------------------------------------
    # Snapshot / restore (used by the undo/redo stack)
    # ------------------------------------------------------------------

    def snapshot(self) -> FieldModel:
        """Return a deep copy of this model.

        The copy is completely independent: mutating the original (or any
        nested container within it) does not affect the snapshot, and vice
        versa.  This is the mechanism the undo stack uses to checkpoint state
        before each :class:`~tools.path_planner.editor.commands.Command` is
        applied.

        :return: A new :class:`FieldModel` with deeply copied field values.
        """
        return copy.deepcopy(self)

    def restore(self, snap: FieldModel) -> None:
        """Overwrite every field of this instance from *snap* in place.

        Existing references to ``self`` remain valid after the call — only the
        field values are replaced.  The undo stack calls this when unwinding a
        command so that the canvas and panels, which hold a reference to the
        live model, automatically see the restored state without needing to be
        re-bound.

        :param snap: A previously captured snapshot (produced by
            :meth:`snapshot`) to restore from.  *snap* itself is not modified.
        """
        self.field_mm = snap.field_mm
        self.map_png = snap.map_png
        self.robot_radius_cm = snap.robot_radius_cm
        self.cost_blind_multiplier = snap.cost_blind_multiplier
        self.start_node = snap.start_node
        self.start_heading_deg = snap.start_heading_deg
        self.nodes = copy.deepcopy(snap.nodes)
        self.node_types = copy.deepcopy(snap.node_types)
        self.line_segments = copy.deepcopy(snap.line_segments)
        self.blind_edges = copy.deepcopy(snap.blind_edges)
        self.obstacles = copy.deepcopy(snap.obstacles)

    # ------------------------------------------------------------------
    # JSON round-trip
    # ------------------------------------------------------------------

    @classmethod
    def from_json_dict(cls, data: dict) -> FieldModel:
        """Build a :class:`FieldModel` from the dict returned by
        :func:`~tools.path_planner.editor.loader.load_field_json`.

        Defensively re-tupleizes ``FIELD_MM``, every ``NODES`` value, and
        every ``LINE_SEGMENTS`` / ``BLIND_EDGES`` / ``OBSTACLES`` entry, so the
        method accepts both loader output (already tuples — the cast is a
        no-op) and :meth:`to_json_dict` output (lists, since JSON has no tuple
        type). That symmetry is what makes
        ``FieldModel.from_json_dict(model.to_json_dict()) == model`` hold.

        Key mapping (SCREAMING_SNAKE_CASE JSON key -> snake_case attribute):

        =================== ====================
        JSON key            Attribute
        =================== ====================
        ``FIELD_MM``        ``field_mm``
        ``MAP_PNG``         ``map_png``
        ``ROBOT_RADIUS_CM`` ``robot_radius_cm``
        ``COST_BLIND_MULTIPLIER`` ``cost_blind_multiplier``
        ``START_NODE``      ``start_node``
        ``START_HEADING_DEG`` ``start_heading_deg``
        ``NODES``           ``nodes``
        ``NODE_TYPES``      ``node_types``
        ``LINE_SEGMENTS``   ``line_segments``
        ``BLIND_EDGES``     ``blind_edges``
        ``OBSTACLES``       ``obstacles``
        =================== ====================

        :param data: Dict as returned by
            :func:`~tools.path_planner.editor.loader.load_field_json`.
            Tuple-typed values must already be tuples (the loader's
            responsibility).
        :return: A fully populated :class:`FieldModel`.
        :raises KeyError: If a required key is absent from *data*.
        :raises TypeError: If a value has an unexpected type (propagated from
            the dataclass constructor).
        """
        return cls(
            field_mm=tuple(data["FIELD_MM"]),
            map_png=data["MAP_PNG"],
            robot_radius_cm=data["ROBOT_RADIUS_CM"],
            cost_blind_multiplier=data["COST_BLIND_MULTIPLIER"],
            start_node=data["START_NODE"],
            start_heading_deg=data["START_HEADING_DEG"],
            nodes={name: tuple(coords) for name, coords in data["NODES"].items()},
            node_types=dict(data["NODE_TYPES"]),
            line_segments=[tuple(seg) for seg in data["LINE_SEGMENTS"]],
            blind_edges=[tuple(edge) for edge in data["BLIND_EDGES"]],
            obstacles=[tuple(obs) for obs in data["OBSTACLES"]],
        )

    # ------------------------------------------------------------------
    # Validator integration
    # ------------------------------------------------------------------

    def to_module_proxy(self) -> Any:
        """Return a :class:`types.SimpleNamespace` whose attributes satisfy
        :func:`tools.path_planner.validator.validate`.

        The validator reads the 11 required attributes listed in
        ``validator._REQUIRED_ATTRS`` via ``getattr``.  This method maps
        every snake_case field of :class:`FieldModel` back to the
        corresponding ``SCREAMING_SNAKE_CASE`` name, preserving the exact
        Python types (tuples stay tuples, dicts stay dicts, lists stay
        lists) so that the validator's ``isinstance`` checks pass without
        any conversion.

        The optional ``EDGES`` attribute is intentionally absent from the
        proxy: the GUI model does not own rich per-edge metadata, and the
        validator treats a missing ``EDGES`` as benign (backward-compatible
        with pre-PR4 fixtures).

        :return: A :class:`types.SimpleNamespace` with exactly the 11
            attributes ``validator._REQUIRED_ATTRS`` lists.
        """
        return types.SimpleNamespace(
            NODES=self.nodes,
            NODE_TYPES=self.node_types,
            LINE_SEGMENTS=self.line_segments,
            BLIND_EDGES=self.blind_edges,
            OBSTACLES=self.obstacles,
            ROBOT_RADIUS_CM=self.robot_radius_cm,
            COST_BLIND_MULTIPLIER=self.cost_blind_multiplier,
            START_NODE=self.start_node,
            START_HEADING_DEG=self.start_heading_deg,
            FIELD_MM=self.field_mm,
            MAP_PNG=self.map_png,
        )

    def validate(self) -> None:
        """Validate the in-memory model against the full PR3/PR4 schema rules.

        Delegates to :func:`tools.path_planner.validator.validate` after
        wrapping ``self`` in a :class:`types.SimpleNamespace` proxy (see
        :meth:`to_module_proxy`).  Imports are deferred inside the method
        body to prevent a circular import: ``validator`` may be imported by
        modules that are themselves imported at the top of this file in the
        future.

        :raises tools.path_planner.validator.FieldDataError: On the first
            schema violation found, with a precise key-path message and a
            one-line fix hint.
        :return: ``None`` on success.
        """
        # Deferred import — keeps module-level import graph acyclic.
        from tools.path_planner.validator import validate  # noqa: PLC0415

        validate(self.to_module_proxy())

    def to_json_dict(self) -> dict:
        """Produce a dict ready for
        :func:`~tools.path_planner.editor.loader.dump_field_json`.

        Tuples are converted back to lists so the output is unambiguously
        JSON-serialisable and the ``dump_field_json`` writer receives the same
        types it would have if reading a freshly parsed JSON file.

        Ordering guarantees:

        - Top-level keys follow ``_JSON_KEY_ORDER`` exactly.
        - ``NODES`` and ``NODE_TYPES`` dict keys are sorted alphabetically.
        - ``LINE_SEGMENTS``, ``BLIND_EDGES``, and ``OBSTACLES`` preserve the
          original list order (spatial order is semantically meaningful for
          the renderer).

        :return: A :class:`dict` whose values are JSON-native types (``str``,
            ``float``, ``int``, ``list``, ``dict``).  No tuples are present.
        """
        return {
            "FIELD_MM": list(self.field_mm),
            "MAP_PNG": self.map_png,
            "ROBOT_RADIUS_CM": float(self.robot_radius_cm),
            "COST_BLIND_MULTIPLIER": float(self.cost_blind_multiplier),
            "START_NODE": self.start_node,
            "START_HEADING_DEG": float(self.start_heading_deg),
            "NODES": {
                name: list(coords)
                for name, coords in sorted(self.nodes.items())
            },
            "NODE_TYPES": {
                name: ntype
                for name, ntype in sorted(self.node_types.items())
            },
            "LINE_SEGMENTS": [list(seg) for seg in self.line_segments],
            "BLIND_EDGES": [list(edge) for edge in self.blind_edges],
            "OBSTACLES": [list(obs) for obs in self.obstacles],
        }
