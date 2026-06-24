"""Tests for the JSON loader/dumper round-trip and error paths.

Covers:
- TC1: load_field_json returns correct tuple types for all validated fields
- TC1 (validator): loaded dict passes validator.validate via a SimpleNamespace proxy
- TC2: load -> dump -> load round-trip produces an equal dict (model-free)
- TC3: dump(load(dump(d))) bytes are identical to dump(d) (idempotency)
- Missing-file error with --migrate hint in message
- Malformed JSON error with line number in message
"""

from __future__ import annotations

import json
import types
from pathlib import Path

import pytest

from tools.path_planner.editor.loader import dump_field_json, load_field_json
from tools.path_planner.validator import FieldDataError, validate

# ---------------------------------------------------------------------------
# Path to the synthetic small fixture checked into the test tree
# ---------------------------------------------------------------------------

_FIXTURES_DIR = Path(__file__).resolve().parent / "fixtures"
_SMALL_FIXTURE = _FIXTURES_DIR / "field_data_small.json"


# ---------------------------------------------------------------------------
# TC1: tuple-type assertions
# ---------------------------------------------------------------------------


def test_tc1_load_returns_tuples_for_validator_fields() -> None:
    """TC1: load_field_json must convert JSON arrays to tuples for every field
    the validator type-checks with isinstance(..., tuple).

    Checked fields:
    - FIELD_MM: tuple of 2 ints/floats
    - NODES values: each is a tuple
    - LINE_SEGMENTS entries: each is a tuple
    - BLIND_EDGES entries: each is a tuple
    - OBSTACLES entries: each is a 5-tuple with kind (str) at index 4
    """
    data = load_field_json(_SMALL_FIXTURE)

    # FIELD_MM must be a tuple of 2 numeric elements.
    assert isinstance(data["FIELD_MM"], tuple), (
        f"FIELD_MM expected tuple, got {type(data['FIELD_MM'])}"
    )
    assert len(data["FIELD_MM"]) == 2

    # Every NODES value must be a tuple.
    assert isinstance(data["NODES"], dict)
    for name, coord in data["NODES"].items():
        assert isinstance(coord, tuple), (
            f"NODES[{name!r}] expected tuple, got {type(coord)}"
        )

    # Every LINE_SEGMENTS entry must be a tuple.
    assert isinstance(data["LINE_SEGMENTS"], list)
    for i, edge in enumerate(data["LINE_SEGMENTS"]):
        assert isinstance(edge, tuple), (
            f"LINE_SEGMENTS[{i}] expected tuple, got {type(edge)}"
        )

    # Every BLIND_EDGES entry must be a tuple.
    assert isinstance(data["BLIND_EDGES"], list)
    for i, edge in enumerate(data["BLIND_EDGES"]):
        assert isinstance(edge, tuple), (
            f"BLIND_EDGES[{i}] expected tuple, got {type(edge)}"
        )

    # Every OBSTACLES entry must be a 5-tuple with a str kind at index 4.
    assert isinstance(data["OBSTACLES"], list)
    for i, obs in enumerate(data["OBSTACLES"]):
        assert isinstance(obs, tuple), (
            f"OBSTACLES[{i}] expected tuple, got {type(obs)}"
        )
        assert len(obs) == 5, (
            f"OBSTACLES[{i}] expected 5-tuple, got length {len(obs)}"
        )
        assert isinstance(obs[4], str), (
            f"OBSTACLES[{i}][4] (kind) expected str, got {type(obs[4])}"
        )


def test_tc1_loaded_dict_passes_validator() -> None:
    """TC1 (validator integration): load_field_json output, wrapped in a
    SimpleNamespace, must pass validator.validate without raising.

    This confirms that the loader's type conversions are exactly what the
    validator expects -- no mismatched types, no missing keys.
    """
    data = load_field_json(_SMALL_FIXTURE)

    # Build a SimpleNamespace whose attributes mirror field_data module attrs.
    proxy = types.SimpleNamespace(**data)

    # Must not raise.
    validate(proxy)


# ---------------------------------------------------------------------------
# TC2: round-trip data equality (model-free)
# ---------------------------------------------------------------------------


def test_tc2_round_trip_data_equality(tmp_path: Path) -> None:
    """TC2 (model-free): load -> dump -> load again produces an equal dict.

    The round-trip is entirely at the loader layer: no FieldModel involved.
    dict equality is used (not repr) so the assertion is type-correct.
    """
    first_load = load_field_json(_SMALL_FIXTURE)

    # Dump the first-loaded dict to a temp file.
    dumped_path = tmp_path / "field_data_rt.json"
    dump_field_json(first_load, dumped_path)

    # Load again from the dumped file.
    second_load = load_field_json(dumped_path)

    assert first_load == second_load, (
        "Round-trip load -> dump -> load produced a different dict"
    )


# ---------------------------------------------------------------------------
# TC3: dump idempotency -- dump(load(dump(d))) bytes == dump(d)
# ---------------------------------------------------------------------------


def test_tc3_dump_is_byte_deterministic(tmp_path: Path) -> None:
    """TC3: dump(load(dump(d))) must be byte-identical to dump(d).

    This locks down the serialization contract: no floating-point
    reformatting, no key reordering, no trailing-whitespace drift.
    """
    first_load = load_field_json(_SMALL_FIXTURE)

    # First dump.
    dump1_path = tmp_path / "dump1.json"
    dump_field_json(first_load, dump1_path)

    # Second dump via load -> dump chain.
    second_load = load_field_json(dump1_path)
    dump2_path = tmp_path / "dump2.json"
    dump_field_json(second_load, dump2_path)

    dump1_bytes = dump1_path.read_bytes()
    dump2_bytes = dump2_path.read_bytes()

    assert dump1_bytes == dump2_bytes, (
        "dump(load(dump(d))) is not byte-identical to dump(d) -- "
        "the serialization is not idempotent"
    )


# ---------------------------------------------------------------------------
# pivot_turns: optional per-turn pivot authoring key round-trips through the
# loader as (u, v, w) tuples, in its canonical slot, byte-deterministically.
# ---------------------------------------------------------------------------


def _small_fixture_with_pivots(tmp_path: Path) -> Path:
    """Write a copy of the small fixture carrying a pivot_turns list.

    The pivot is authored as a 3-element node-name array, the same
    array-of-arrays shape LINE_SEGMENTS / BLIND_EDGES use. ('A', 'B', 'G')
    are nodes the small fixture already defines, so the loaded dict stays
    validator-clean independently of this test's focus on tuple discipline.
    """
    data = json.loads(_SMALL_FIXTURE.read_text(encoding="utf-8"))
    # Insert between BLIND_EDGES and OBSTACLES to mirror the canonical order;
    # the loader re-orders on dump regardless, so input order is not load-bearing.
    data["pivot_turns"] = [["A", "B", "G"]]
    target = tmp_path / "field_data_with_pivots.json"
    target.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8", newline="\n")
    return target


def test_pivot_turns_loads_as_three_tuples(tmp_path: Path) -> None:
    """load_field_json converts each pivot_turns 3-array to a (u, v, w) tuple.

    Mirrors the LINE_SEGMENTS / BLIND_EDGES tuple discipline: JSON has no
    tuple type, so the loader must materialise tuples for the downstream
    consumers that compare and hash these triples.
    """
    fixture = _small_fixture_with_pivots(tmp_path)
    data = load_field_json(fixture)

    assert isinstance(data["pivot_turns"], list)
    assert len(data["pivot_turns"]) == 1
    turn = data["pivot_turns"][0]
    assert isinstance(turn, tuple), f"pivot_turns[0] expected tuple, got {type(turn)}"
    assert turn == ("A", "B", "G")


def test_pivot_turns_round_trip_data_equality(tmp_path: Path) -> None:
    """load -> dump -> load preserves pivot_turns as an equal list of tuples."""
    fixture = _small_fixture_with_pivots(tmp_path)
    first_load = load_field_json(fixture)

    dumped_path = tmp_path / "field_data_pivots_rt.json"
    dump_field_json(first_load, dumped_path)
    second_load = load_field_json(dumped_path)

    assert first_load == second_load
    assert second_load["pivot_turns"] == [("A", "B", "G")]


def test_pivot_turns_dumped_in_canonical_slot(tmp_path: Path) -> None:
    """dump_field_json emits pivot_turns between BLIND_EDGES and OBSTACLES."""
    fixture = _small_fixture_with_pivots(tmp_path)
    first_load = load_field_json(fixture)

    dumped_path = tmp_path / "field_data_pivots_order.json"
    dump_field_json(first_load, dumped_path)
    keys = list(json.loads(dumped_path.read_text(encoding="utf-8")).keys())

    assert "pivot_turns" in keys
    assert keys.index("BLIND_EDGES") < keys.index("pivot_turns") < keys.index("OBSTACLES")


def test_pivot_turns_absent_round_trips_absent(tmp_path: Path) -> None:
    """A field that never authored pivot_turns round-trips without the key.

    The small fixture omits pivot_turns; the loader must not synthesise it and
    the dump must not inject an empty list, so dump(load(x)) == dump(x) holds
    byte-for-byte for a pivot-free input (absent stays absent).
    """
    first_load = load_field_json(_SMALL_FIXTURE)
    assert "pivot_turns" not in first_load

    dumped_path = tmp_path / "field_data_no_pivots.json"
    dump_field_json(first_load, dumped_path)
    second_load = load_field_json(dumped_path)

    assert "pivot_turns" not in second_load
    assert first_load == second_load


def test_pivot_turns_present_dump_is_byte_deterministic(tmp_path: Path) -> None:
    """dump(load(dump(d))) is byte-identical to dump(d) when pivot_turns is set."""
    fixture = _small_fixture_with_pivots(tmp_path)
    first_load = load_field_json(fixture)

    dump1_path = tmp_path / "pivots_dump1.json"
    dump_field_json(first_load, dump1_path)

    second_load = load_field_json(dump1_path)
    dump2_path = tmp_path / "pivots_dump2.json"
    dump_field_json(second_load, dump2_path)

    assert dump1_path.read_bytes() == dump2_path.read_bytes()


# ---------------------------------------------------------------------------
# Issue #79 / ADR-047: EDGES override overlay + the four pipeline_* mode keys
# round-trip through the loader in their canonical slots, byte-deterministically,
# and stay ABSENT when the input never authored them (absent-stays-absent).
# ---------------------------------------------------------------------------


def _small_fixture_with_edges_and_pipeline(tmp_path: Path) -> Path:
    """Write a copy of the small fixture carrying EDGES + all four pipeline keys.

    The EDGES overlay attaches a string-tag override to the existing A-B LINE
    pair (an override-only entry, no geometry) plus a geometry-bearing entry on
    the B-G BLIND pair (the backward-compat form). The four ``pipeline_*`` keys
    carry concrete enum member NAMES. ('A', 'B', 'G') are nodes the small
    fixture already defines, so the loaded dict stays well-formed.
    """
    data = json.loads(_SMALL_FIXTURE.read_text(encoding="utf-8"))
    data["pipeline_line_mode"] = "FAST"
    data["pipeline_blind_mode"] = "GYRO_TSHAPE"
    data["pipeline_turn_mode"] = "PIVOT_LEFT"
    data["pipeline_direction_mode"] = "DEFAULT"
    data["EDGES"] = [
        {"type": "LINE", "u": "A", "v": "B", "primitive_mode": "FAST"},
        {
            "type": "BLIND",
            "u": "B",
            "v": "G",
            "distance_cm": 86.0,
            "heading_deg": 54.46,
            "stop_at_line": "on",
        },
    ]
    target = tmp_path / "field_data_with_edges_pipeline.json"
    target.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8", newline="\n")
    return target


def test_tc10_edges_and_pipeline_round_trip_data_equality(tmp_path: Path) -> None:
    """TC10: load -> dump -> load preserves EDGES + pipeline keys as an equal dict.

    EDGES is a list of plain dicts (not array-of-arrays), so the loader must NOT
    tuple-convert it; the entries pass through verbatim, including the
    geometry-bearing one. The four pipeline keys are plain strings and pass
    through unchanged.
    """
    fixture = _small_fixture_with_edges_and_pipeline(tmp_path)
    first_load = load_field_json(fixture)

    # EDGES entries are dicts, NOT tuples (the tuple discipline does not apply).
    assert isinstance(first_load["EDGES"], list)
    for entry in first_load["EDGES"]:
        assert isinstance(entry, dict)
    assert first_load["pipeline_line_mode"] == "FAST"
    assert first_load["pipeline_blind_mode"] == "GYRO_TSHAPE"
    assert first_load["pipeline_turn_mode"] == "PIVOT_LEFT"
    assert first_load["pipeline_direction_mode"] == "DEFAULT"

    dumped_path = tmp_path / "field_data_edges_rt.json"
    dump_field_json(first_load, dumped_path)
    second_load = load_field_json(dumped_path)

    assert first_load == second_load
    # The geometry-bearing entry round-trips verbatim (not normalized / dropped).
    blind_entry = next(e for e in second_load["EDGES"] if e["type"] == "BLIND")
    assert blind_entry["distance_cm"] == 86.0
    assert blind_entry["stop_at_line"] == "on"


def test_tc10_edges_and_pipeline_dump_is_byte_deterministic(tmp_path: Path) -> None:
    """TC10: dump(load(dump(d))) is byte-identical when EDGES + pipeline are set."""
    fixture = _small_fixture_with_edges_and_pipeline(tmp_path)
    first_load = load_field_json(fixture)

    dump1_path = tmp_path / "edges_dump1.json"
    dump_field_json(first_load, dump1_path)

    second_load = load_field_json(dump1_path)
    dump2_path = tmp_path / "edges_dump2.json"
    dump_field_json(second_load, dump2_path)

    assert dump1_path.read_bytes() == dump2_path.read_bytes()


def test_tc10_edges_and_pipeline_dumped_in_canonical_slots(tmp_path: Path) -> None:
    """TC10: the new keys land in their pinned slots before OBSTACLES.

    The contract pins the order: the four ``pipeline_*`` keys, then ``EDGES``,
    then ``OBSTACLES`` (all after ``blind_projection_auto_seek`` /
    ``BLIND_EDGES``). The loader re-orders on dump regardless of input order.
    """
    fixture = _small_fixture_with_edges_and_pipeline(tmp_path)
    first_load = load_field_json(fixture)

    dumped_path = tmp_path / "edges_order.json"
    dump_field_json(first_load, dumped_path)
    keys = list(json.loads(dumped_path.read_text(encoding="utf-8")).keys())

    # Pipeline keys precede EDGES, EDGES precedes OBSTACLES.
    assert (
        keys.index("pipeline_line_mode")
        < keys.index("pipeline_blind_mode")
        < keys.index("pipeline_turn_mode")
        < keys.index("pipeline_direction_mode")
        < keys.index("EDGES")
        < keys.index("OBSTACLES")
    )


def test_tc11_new_keys_absent_round_trip_absent(tmp_path: Path) -> None:
    """TC11: a field WITHOUT the new keys round-trips with them still absent.

    The small fixture omits EDGES and all four pipeline keys; the loader must not
    synthesise them and the dump must not inject empty placeholders, so
    dump(load(x)) == dump(x) holds byte-for-byte (absent stays absent).
    """
    first_load = load_field_json(_SMALL_FIXTURE)
    for key in (
        "EDGES",
        "pipeline_line_mode",
        "pipeline_blind_mode",
        "pipeline_turn_mode",
        "pipeline_direction_mode",
    ):
        assert key not in first_load

    dumped_path = tmp_path / "no_new_keys.json"
    dump_field_json(first_load, dumped_path)
    second_load = load_field_json(dumped_path)

    for key in (
        "EDGES",
        "pipeline_line_mode",
        "pipeline_blind_mode",
        "pipeline_turn_mode",
        "pipeline_direction_mode",
    ):
        assert key not in second_load
    assert first_load == second_load


# ---------------------------------------------------------------------------
# Issue #79 / ADR-047: FieldModel dual-serialization parity (TC12, the #1 risk).
#
# A field that lands in to_json_dict but NOT to_module_proxy (or vice versa) is
# persisted-but-unvalidated. The model must carry EDGES + PIPELINE_* through BOTH
# surfaces; this proves the validator runs against the edited values AND both
# surfaces emit the same logical key set.
# ---------------------------------------------------------------------------


def test_tc12_model_dual_serialization_parity() -> None:
    """TC12: to_module_proxy() AND to_json_dict() BOTH carry EDGES + PIPELINE_*.

    The dual-serialization seam (the #1 risk). Build a model with an EDGES
    override and all four pipeline keys, then assert:

    1. ``to_module_proxy()`` exposes EDGES + the SCREAMING_SNAKE PIPELINE_*
       attributes so the validator sees the edited values (validate runs clean);
    2. an INVALID edited value (a DEFAULT pipeline_line_mode) is actually CAUGHT
       through the proxy -- proving the proxy is wired into validate(), not inert;
    3. ``to_json_dict()`` emits EDGES (snake) + the four pipeline keys; and
    4. both surfaces carry the SAME logical key set (proxy SCREAMING_SNAKE <->
       json snake), so neither persists a value the other never validates.
    """
    from tools.path_planner.editor.model import FieldModel

    model = FieldModel(
        field_mm=(3000, 2000),
        map_png="maps/map.png",
        robot_radius_cm=8.0,
        blind_decay_enabled=True,
        blind_decay_k=0.05,
        blind_decay_exponent=2.0,
        blind_decay_d_ref=50.0,
        start_node="S",
        start_heading_deg=0.0,
        nodes={"S": (10.0, 100.0), "A": (10.0, 10.0), "B": (200.0, 10.0)},
        node_types={"S": "blind", "A": "line", "B": "line"},
        line_segments=[("A", "B")],
        blind_edges=[],
        obstacles=[],
        edges=[{"type": "LINE", "u": "A", "v": "B", "primitive_mode": "FAST"}],
        pipeline_line_mode="FAST",
        pipeline_blind_mode="GYRO_TSHAPE",
        pipeline_turn_mode="PIVOT_LEFT",
        pipeline_direction_mode="DEFAULT",
    )

    # (1) The proxy exposes the edited values, and validate() runs clean.
    proxy = model.to_module_proxy()
    assert proxy.EDGES == [
        {"type": "LINE", "u": "A", "v": "B", "primitive_mode": "FAST"}
    ]
    assert proxy.PIPELINE_LINE_MODE == "FAST"
    assert proxy.PIPELINE_BLIND_MODE == "GYRO_TSHAPE"
    assert proxy.PIPELINE_TURN_MODE == "PIVOT_LEFT"
    assert proxy.PIPELINE_DIRECTION_MODE == "DEFAULT"
    model.validate()  # must not raise -- the edited values are checked

    # (2) An invalid edited pipeline value is CAUGHT through the proxy, proving
    # the proxy is actually wired into validate (not a persisted-but-unvalidated
    # field). line may not be DEFAULT (circular).
    model.pipeline_line_mode = "DEFAULT"
    with pytest.raises(FieldDataError, match="PIPELINE_LINE_MODE"):
        model.validate()
    model.pipeline_line_mode = "FAST"  # restore for the serialization checks

    # (3) to_json_dict emits EDGES (snake) + the four pipeline keys.
    json_dict = model.to_json_dict()
    assert json_dict["EDGES"] == [
        {"type": "LINE", "u": "A", "v": "B", "primitive_mode": "FAST"}
    ]
    assert json_dict["pipeline_line_mode"] == "FAST"
    assert json_dict["pipeline_blind_mode"] == "GYRO_TSHAPE"
    assert json_dict["pipeline_turn_mode"] == "PIVOT_LEFT"
    assert json_dict["pipeline_direction_mode"] == "DEFAULT"

    # (4) Both surfaces carry the SAME logical key set (the dual-serialization
    # parity guarantee): the JSON snake keys map 1:1 onto the proxy SCREAMING_SNAKE
    # attributes, so no surface persists a key the other never validates.
    json_to_proxy = {
        "EDGES": "EDGES",
        "pipeline_line_mode": "PIPELINE_LINE_MODE",
        "pipeline_blind_mode": "PIPELINE_BLIND_MODE",
        "pipeline_turn_mode": "PIPELINE_TURN_MODE",
        "pipeline_direction_mode": "PIPELINE_DIRECTION_MODE",
    }
    for json_key, proxy_attr in json_to_proxy.items():
        assert json_key in json_dict, f"{json_key} missing from to_json_dict"
        assert hasattr(proxy, proxy_attr), f"{proxy_attr} missing from proxy"
        assert json_dict[json_key] == getattr(proxy, proxy_attr)


# ---------------------------------------------------------------------------
# Error path: missing file must mention --migrate
# ---------------------------------------------------------------------------


def test_load_missing_file_raises_field_data_error_with_migration_hint(
    tmp_path: Path,
) -> None:
    """load_field_json pointed at a non-existent path must raise FieldDataError
    whose message contains '--migrate' so the user knows how to fix it.
    """
    missing = tmp_path / "no_such_file.json"

    with pytest.raises(FieldDataError) as exc_info:
        load_field_json(missing)

    assert "--migrate" in str(exc_info.value), (
        f"Expected '--migrate' in error message; got: {exc_info.value}"
    )


# ---------------------------------------------------------------------------
# Error path: malformed JSON must report line number
# ---------------------------------------------------------------------------


def test_load_malformed_json_raises_field_data_error_with_lineno(
    tmp_path: Path,
) -> None:
    """load_field_json on a file with invalid JSON must raise FieldDataError
    whose message contains 'line' (pointing at the offending line number).
    """
    bad_json = tmp_path / "bad.json"
    # Trailing comma after the last value makes this invalid JSON.
    bad_json.write_text(
        '{\n  "FIELD_MM": [3000, 2000],\n}\n',
        encoding="utf-8",
        newline="\n",
    )

    with pytest.raises(FieldDataError) as exc_info:
        load_field_json(bad_json)

    assert "line" in str(exc_info.value).lower(), (
        f"Expected 'line' in error message; got: {exc_info.value}"
    )
