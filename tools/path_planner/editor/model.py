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
from dataclasses import dataclass, field
from typing import Any, ClassVar

# Canonical authoring vocabularies, single-sourced here (the lightest editor
# module, a leaf in the import graph) so the GUI combobox values, the inline
# guards, and the command validators cannot drift apart and accept-in-GUI /
# reject-at-validate. They mirror the planner's validator (NODE_TYPES are
# line/blind/reset; reset axes mirror ``validator._ALLOWED_RESET_AXES``); the
# Save-time ``validate()`` is the ultimate authority and re-checks both.
NODE_TYPE_VALUES: tuple[str, ...] = ("line", "blind", "reset")
RESET_AXIS_VALUES: tuple[str, ...] = ("x", "y")


@dataclass
class FieldModel:
    """Mirror of the ``field_data.json`` schema held in memory while editing.

    :param field_mm: Outer field dimensions ``(width_mm, height_mm)``.
    :param map_png: Relative path (from ``tools/path_planner/``) to the
        background PNG used by the visualizer.
    :param robot_radius_cm: Inflation radius in centimetres applied to each
        obstacle before edge-intersection checks.
    :param blind_decay_enabled: Whether the blind-distance decay penalty is
        applied to blind-edge weights at table-build time.
    :param blind_decay_k: Decay weight ``k`` (unitless, ``>= 0``) in
        ``weight = length * (1 + k * (d / d_ref) ** exponent)``. At ``k == 0``
        the penalty vanishes and blind weights reduce to plain length.
    :param blind_decay_exponent: Decay exponent (``>= 1``) shaping
        ``g(d) = (d / d_ref) ** exponent``.
    :param blind_decay_d_ref: Decay normalizer ``d_ref`` in centimetres
        (``> 0``) so ``k`` is map-portable.
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
    :param pivot_turns: Per-turn single-wheel pivots, each as a directed
        ``(u, v, w)`` node-name triple (arrive from ``u``, pivot at ``v``,
        leave to ``w``). The editor has no GUI control for these; the field
        is carried verbatim so an open -> save round-trip preserves any
        pivots authored directly in ``field_data.json``. Defaults to an empty
        list when the JSON omits the optional ``pivot_turns`` key.
    :param reset_nodes: Wall-zero reset nodes, a sparse table keyed by node
        name mapping each ``"reset"``-typed node to its
        ``{"axis", "wall_coord", "facing"}`` re-localization spec. Like
        ``pivot_turns``, the editor has no GUI control for these; the table is
        carried verbatim so an open -> save round-trip preserves any reset
        nodes authored directly in ``field_data.json``. Defaults to an empty
        dict when the JSON omits the optional ``RESET_NODES`` key.
    """

    # Ordered list of top-level JSON keys.  ``to_json_dict`` produces keys in
    # this exact order so the output file is deterministic across Python
    # versions and dict implementations.
    _JSON_KEY_ORDER: ClassVar[tuple[str, ...]] = (
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
        "pivot_turns",
        "RESET_NODES",
        "blind_projection_auto_seek",
        "OBSTACLES",
    )

    field_mm: tuple[float, float]
    map_png: str
    robot_radius_cm: float
    blind_decay_enabled: bool
    blind_decay_k: float
    blind_decay_exponent: float
    blind_decay_d_ref: float
    start_node: str
    start_heading_deg: float
    nodes: dict[str, tuple[float, float]]
    node_types: dict[str, str]
    line_segments: list[tuple[str, str]]
    blind_edges: list[tuple[str, str]]
    obstacles: list[tuple[float, float, float, float, str]]
    # Optional, preserved-only (no GUI control). Defaulted so existing call
    # sites that construct a FieldModel without pivots stay valid; placed last
    # so the defaulted field follows the required ones.
    pivot_turns: list[tuple[str, str, str]] = field(default_factory=list)
    # Optional, preserved-only (no GUI control). Sparse table keyed by node
    # name -> {axis, wall_coord, facing}. Defaulted so existing call sites that
    # build a FieldModel without reset nodes stay valid.
    reset_nodes: dict[str, dict] = field(default_factory=dict)
    # Optional field-wide flag (default True). Whether a BLIND hop that precedes
    # a PROJECTION auto-emits a stop-at-line seek (ADR-037,
    # BLIND_PROJECTION_AUTO_SEEK in field_data). Tracked here so an
    # open -> save round-trip preserves a field that set it False; without it
    # the editor's Save dropped the key and field_data.py's loader silently
    # reverted the flag to its True default.
    blind_projection_auto_seek: bool = True

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
        self.blind_decay_enabled = snap.blind_decay_enabled
        self.blind_decay_k = snap.blind_decay_k
        self.blind_decay_exponent = snap.blind_decay_exponent
        self.blind_decay_d_ref = snap.blind_decay_d_ref
        self.start_node = snap.start_node
        self.start_heading_deg = snap.start_heading_deg
        self.nodes = copy.deepcopy(snap.nodes)
        self.node_types = copy.deepcopy(snap.node_types)
        self.line_segments = copy.deepcopy(snap.line_segments)
        self.blind_edges = copy.deepcopy(snap.blind_edges)
        self.obstacles = copy.deepcopy(snap.obstacles)
        self.pivot_turns = copy.deepcopy(snap.pivot_turns)
        self.reset_nodes = copy.deepcopy(snap.reset_nodes)
        self.blind_projection_auto_seek = snap.blind_projection_auto_seek

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
        ``BLIND_DECAY_ENABLED`` ``blind_decay_enabled``
        ``BLIND_DECAY_K``   ``blind_decay_k``
        ``BLIND_DECAY_EXPONENT`` ``blind_decay_exponent``
        ``BLIND_DECAY_D_REF`` ``blind_decay_d_ref``
        ``START_NODE``      ``start_node``
        ``START_HEADING_DEG`` ``start_heading_deg``
        ``NODES``           ``nodes``
        ``NODE_TYPES``      ``node_types``
        ``LINE_SEGMENTS``   ``line_segments``
        ``BLIND_EDGES``     ``blind_edges``
        ``pivot_turns``     ``pivot_turns``
        ``RESET_NODES``     ``reset_nodes``
        ``OBSTACLES``       ``obstacles``
        =================== ====================

        ``pivot_turns`` is optional (older JSON files predate it): an absent
        key maps to an empty list, and each present 3-array becomes a
        ``(u, v, w)`` tuple, mirroring the ``LINE_SEGMENTS`` treatment.

        ``RESET_NODES`` is optional too (older JSON files predate it): an
        absent key maps to an empty dict, and each present per-node spec is
        deep-copied so the model owns an independent table (its values are
        plain JSON objects, so no tuple conversion applies).

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
            blind_decay_enabled=data["BLIND_DECAY_ENABLED"],
            blind_decay_k=data["BLIND_DECAY_K"],
            blind_decay_exponent=data["BLIND_DECAY_EXPONENT"],
            blind_decay_d_ref=data["BLIND_DECAY_D_REF"],
            start_node=data["START_NODE"],
            start_heading_deg=data["START_HEADING_DEG"],
            nodes={name: tuple(coords) for name, coords in data["NODES"].items()},
            node_types=dict(data["NODE_TYPES"]),
            line_segments=[tuple(seg) for seg in data["LINE_SEGMENTS"]],
            blind_edges=[tuple(edge) for edge in data["BLIND_EDGES"]],
            obstacles=[tuple(obs) for obs in data["OBSTACLES"]],
            pivot_turns=[tuple(turn) for turn in data.get("pivot_turns", [])],
            reset_nodes={
                name: dict(spec)
                for name, spec in data.get("RESET_NODES", {}).items()
            },
            blind_projection_auto_seek=data.get(
                "blind_projection_auto_seek", True
            ),
        )

    # ------------------------------------------------------------------
    # Validator integration
    # ------------------------------------------------------------------

    def to_module_proxy(self) -> Any:
        """Return a :class:`types.SimpleNamespace` whose attributes satisfy
        :func:`tools.path_planner.validator.validate`.

        The validator reads the 14 required attributes listed in
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

        ``PIVOT_TURNS`` IS exposed: the model persists it via
        :meth:`to_json_dict`, so the proxy must carry it too or
        :func:`validator._validate_pivot_turns` hits its
        ``getattr(..., 'PIVOT_TURNS', None)`` early-return and the editor's
        Save silently skips every pivot geometry rule (the docstring's "full
        PR3/PR4 schema rules" promise would not hold for pivots).

        ``RESET_NODES`` IS exposed for the same reason: the model persists it
        via :meth:`to_json_dict`, so the proxy must carry it too or
        :func:`validator._validate_reset_nodes` hits its
        ``getattr(..., 'RESET_NODES', {})`` early-return and the editor's Save
        silently skips every reset-node rule (the dual-serialization trap --
        a field added to JSON but not the proxy is persisted-but-unvalidated).

        ``BLIND_PROJECTION_AUTO_SEEK`` IS exposed too: the model persists it via
        :meth:`to_json_dict`, so the proxy must carry it or
        :func:`validator.validate`'s ``hasattr(..., 'BLIND_PROJECTION_AUTO_SEEK')``
        type-check never runs against the edited value (the same
        dual-serialization trap).

        :return: A :class:`types.SimpleNamespace` with the 14 required
            attributes ``validator._REQUIRED_ATTRS`` lists plus the optional
            ``PIVOT_TURNS``, ``RESET_NODES``, and ``BLIND_PROJECTION_AUTO_SEEK``.
        """
        return types.SimpleNamespace(
            NODES=self.nodes,
            NODE_TYPES=self.node_types,
            LINE_SEGMENTS=self.line_segments,
            BLIND_EDGES=self.blind_edges,
            OBSTACLES=self.obstacles,
            ROBOT_RADIUS_CM=self.robot_radius_cm,
            BLIND_DECAY_ENABLED=self.blind_decay_enabled,
            BLIND_DECAY_K=self.blind_decay_k,
            BLIND_DECAY_EXPONENT=self.blind_decay_exponent,
            BLIND_DECAY_D_REF=self.blind_decay_d_ref,
            START_NODE=self.start_node,
            START_HEADING_DEG=self.start_heading_deg,
            FIELD_MM=self.field_mm,
            MAP_PNG=self.map_png,
            PIVOT_TURNS=self.pivot_turns,
            RESET_NODES=self.reset_nodes,
            BLIND_PROJECTION_AUTO_SEEK=self.blind_projection_auto_seek,
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
        - ``LINE_SEGMENTS``, ``BLIND_EDGES``, ``pivot_turns``, and
          ``OBSTACLES`` preserve the original list order (spatial / authoring
          order is semantically meaningful for the renderer and the planner).
        - ``RESET_NODES`` dict keys are sorted alphabetically (like ``NODES``
          / ``NODE_TYPES``) so the dump is byte-deterministic.
        - ``pivot_turns`` is OPTIONAL: it is emitted (in its canonical slot,
          between ``BLIND_EDGES`` and ``RESET_NODES``) only when non-empty. A
          pivot-free field omits the key entirely, matching the loader's
          ``dump_field_json`` "absent stays absent" round-trip so a save
          through the model produces no spurious ``pivot_turns: []`` diff.
        - ``RESET_NODES`` is OPTIONAL too: it is emitted (in its canonical
          slot, between ``pivot_turns`` and ``OBSTACLES``) only when non-empty.
          A reset-free field omits the key entirely, matching the same
          "absent stays absent" round-trip so a save produces no spurious
          ``RESET_NODES: {}`` diff.

        :return: A :class:`dict` whose values are JSON-native types (``str``,
            ``float``, ``int``, ``list``, ``dict``).  No tuples are present.
        """
        result = {
            "FIELD_MM": list(self.field_mm),
            "MAP_PNG": self.map_png,
            "ROBOT_RADIUS_CM": float(self.robot_radius_cm),
            "BLIND_DECAY_ENABLED": bool(self.blind_decay_enabled),
            "BLIND_DECAY_K": float(self.blind_decay_k),
            "BLIND_DECAY_EXPONENT": float(self.blind_decay_exponent),
            "BLIND_DECAY_D_REF": float(self.blind_decay_d_ref),
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
        }
        if self.pivot_turns:
            result["pivot_turns"] = [list(turn) for turn in self.pivot_turns]
        if self.reset_nodes:
            result["RESET_NODES"] = {
                name: dict(self.reset_nodes[name])
                for name in sorted(self.reset_nodes)
            }
        # OPTIONAL: emit only when it differs from the loader default (True), so
        # a field that never authored the key round-trips byte-identically
        # (absent stays absent), matching pivot_turns / RESET_NODES.
        if not self.blind_projection_auto_seek:
            result["blind_projection_auto_seek"] = False
        result["OBSTACLES"] = [list(obs) for obs in self.obstacles]
        return result
