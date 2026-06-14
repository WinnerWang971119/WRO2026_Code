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
