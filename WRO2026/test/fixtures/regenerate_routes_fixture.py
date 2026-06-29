#!/usr/bin/env python3
"""Regenerate the host-test fixture routes (``routes_fixture.{h,cpp}``).

This script emits the STABLE, feature-complete fixture graph the firmware FSM-logic
host tests link instead of the production ``WRO2026/routes.cpp``. The field itself
lives in :mod:`field_fixture` (single-sourced so the PathTracker sim<->firmware
parity pytest can import the exact same map); this script only runs that field
through the real emit pipeline and writes the committed headers.

See :mod:`field_fixture` for the field definition, the issue #91 motivation, and
the sixteen-name stable contract the host tests resolve by name.

The emitted ``routes_fixture.h`` carries the production ``ROUTES_H`` include guard
and the identical ``Routes::`` ABI, so the CMake harness force-includes it ahead
of the sketch ``routes.h`` to shadow the production data for those targets only.

The emit pipeline reads the real ``Config.h`` track width
(``parse_track_cm_from_config()``) and runs ``build_pivot_turns()``, so the
committed fixture reflects the live track geometry. The ``Source SHA256:`` stamp
anchors on ``field_fixture.py``: editing that field (or the routes schema) restales
the committed headers, which is the signal to re-run this script.

Run from the repo root::

    uv run --no-sync python WRO2026/test/fixtures/regenerate_routes_fixture.py
"""

from __future__ import annotations

import sys
import warnings
from pathlib import Path

_THIS_DIR = Path(__file__).resolve().parent
_REPO_ROOT = _THIS_DIR.parents[2]  # fixtures -> test -> WRO2026 -> repo root
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from field_fixture import build_field_data  # noqa: E402  (sibling module, same dir)
from tools.path_planner.graph_builder import build_graph  # noqa: E402
from tools.path_planner.header_writer import emit_routes  # noqa: E402
from tools.path_planner.planner import (  # noqa: E402
    BlindDecayParams,
    build_pivot_turns,
    compute_all_pairs_next_hop,
    parse_track_cm_from_config,
)
from tools.path_planner.validator import validate  # noqa: E402

_FIXTURE_HEADER = _THIS_DIR / "routes_fixture.h"
_FIXTURE_SOURCE = _THIS_DIR / "routes_fixture.cpp"
# The field now lives in field_fixture.py, so that module is the SHA stamp anchor:
# a field edit there restales the committed fixture (the intended re-coupling).
_FIELD_FIXTURE_MODULE = _THIS_DIR / "field_fixture.py"


def regenerate() -> None:
    fd = build_field_data()
    # Validate loudly: a bad fixture field must fail here, not emit a silently
    # broken routes_fixture.cpp.
    validate(fd)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        graph = build_graph(fd)
    next_hop = compute_all_pairs_next_hop(graph, BlindDecayParams.from_field_data(fd))
    track_cm = parse_track_cm_from_config()
    pivot_records = build_pivot_turns(graph, track_cm, pivot_turns=fd.PIVOT_TURNS)
    emit_routes(
        fd,
        graph,
        next_hop,
        pivot_records=pivot_records,
        header_path=_FIXTURE_HEADER,
        source_path=_FIXTURE_SOURCE,
        field_data_source_path=_FIELD_FIXTURE_MODULE,
    )
    print("Regenerated host-test fixture routes:")
    print(f"  {_FIXTURE_HEADER}")
    print(f"  {_FIXTURE_SOURCE}")


if __name__ == "__main__":
    regenerate()
