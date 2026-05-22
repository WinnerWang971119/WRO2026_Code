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
