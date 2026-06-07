"""CTPS one-command orchestrator — build, dump, render, open.

Run as a module from the repo root::

    python -m tools.path_planner.ctps
    python -m tools.path_planner.ctps --fixture
    python -m tools.path_planner.ctps --trace run.json --no-open

This is the single front door for the Clear Trajectory Prediction Simulator.
It wires the three pure CTPS modules together end-to-end:

#. Produce a Data-Model-A command-stream JSON. By default this means a host
   C++ build of the ``ctps_dump`` harness (see ``WRO2026/test/CMakeLists.txt``)
   driven by the real ``buildMission``; with ``--fixture`` the synthetic
   fixture mission is dumped instead; with ``--trace FILE`` the build is
   skipped entirely and an existing dump JSON is loaded.
#. ``trace_loader.load`` the JSON into a validated :class:`Trace`.
#. ``track_builder.build`` the trace against the LIVE
   :mod:`tools.path_planner.field_data` into a playback :class:`Track`.
#. ``player_html.render`` the track into one self-contained ``.html`` page.
#. Open the page in the default browser (unless ``--no-open``) and print the
   output path.

Toolchain policy (CTPS plan, Error Handling): the Python side NEVER requires a
C++ compiler when ``--trace`` is supplied. The compiler is located lazily, only
on the default / ``--fixture`` build path, via ``vswhere.exe`` ->
``vcvars64.bat``. A missing toolchain is reported with a clear message that
points the user at ``--trace <existing_dump.json>`` and exits non-zero.

Stdlib only — no third-party dependencies.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import webbrowser
from pathlib import Path
from typing import List, Optional

# When invoked as ``python -m tools.path_planner.ctps`` the package is already
# importable; the ``-m`` launcher puts the cwd (repo root) on ``sys.path``. We
# still resolve the repo root from this file's location so the cmake/build
# invocation does not depend on the caller's cwd.
#
# File layout: tools/path_planner/ctps/__main__.py
#   parents[0] = .../tools/path_planner/ctps
#   parents[1] = .../tools/path_planner
#   parents[2] = .../tools
#   parents[3] = repo (worktree) root
_REPO_ROOT: Path = Path(__file__).resolve().parents[3]

if __name__ == "__main__" and __package__ in (None, ""):
    # Allow ``python tools/path_planner/ctps/__main__.py`` too: prepend the
    # repo root so ``from tools.path_planner...`` resolves. No-op under ``-m``.
    if str(_REPO_ROOT) not in sys.path:
        sys.path.insert(0, str(_REPO_ROOT))

from tools.path_planner import field_data  # noqa: E402
from tools.path_planner.ctps import player_html, track_builder, trace_loader  # noqa: E402

# ---------------------------------------------------------------------------
# Constants — paths and the C++ harness wiring.
# ---------------------------------------------------------------------------

#: Default output path for the rendered HTML player, relative to the repo root.
_DEFAULT_OUT: Path = _REPO_ROOT / "tools" / "path_planner" / "output" / "ctps.html"

#: path_planner root. ``field_data.MAP_PNG`` (e.g. "maps/map.png") resolves
#: relative to this, matching the visualizer's map resolution so the player's
#: embedded background and the static map render agree.
_PLANNER_ROOT: Path = _REPO_ROOT / "tools" / "path_planner"

#: CMake source dir for the host test harness (holds the ``ctps_dump`` target).
_TEST_DIR: Path = _REPO_ROOT / "WRO2026" / "test"

#: Out-of-tree build dir for the ``ctps_dump`` target (gitignored by T11).
_BUILD_DIR: Path = _REPO_ROOT / "build" / "ctps"

#: Built dump executable (Windows). ``cmd /c`` invokes it after the build.
_DUMP_EXE: Path = _BUILD_DIR / "ctps_dump.exe"

#: CMake target built by the configure/build step.
_DUMP_TARGET: str = "ctps_dump"

#: Located via ``%ProgramFiles(x86)%`` so it works under any drive/install.
_VSWHERE: Path = (
    Path(os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)"))
    / "Microsoft Visual Studio"
    / "Installer"
    / "vswhere.exe"
)


# ---------------------------------------------------------------------------
# Toolchain discovery (default / --fixture path only).
# ---------------------------------------------------------------------------


def _toolchain_hint() -> str:
    """Return the shared fallback line pointing the user at ``--trace``.

    :return: a single actionable sentence reused by every toolchain / build
        error so the user always sees the same escape hatch.
    """
    return (
        "Fall back to a pre-recorded dump with: "
        "python -m tools.path_planner.ctps --trace <existing_dump.json>"
    )


def _find_vcvars() -> Optional[Path]:
    """Locate ``vcvars64.bat`` via ``vswhere.exe``.

    Runs ``vswhere -latest -products * -requires <VC tools> -property
    installationPath`` (``-products *`` so standalone Build Tools installs are
    not filtered out) and joins the reported install root with
    ``VC/Auxiliary/Build/vcvars64.bat``. Returns ``None`` when
    ``vswhere`` is absent, fails, reports nothing, or the resolved batch file
    does not exist — every miss is a clean ``None`` so the caller emits one
    unified "no toolchain" message.

    :return: the resolved path to ``vcvars64.bat``, or ``None`` if not found.
    """
    if not _VSWHERE.is_file():
        return None

    try:
        completed = subprocess.run(
            [
                str(_VSWHERE),
                "-latest",
                # -products * is REQUIRED: vswhere excludes standalone
                # "Build Tools" installs by default, returning an empty
                # installationPath on machines without full Visual Studio.
                "-products",
                "*",
                "-requires",
                "Microsoft.VisualStudio.Component.VC.Tools.x86.x64",
                "-property",
                "installationPath",
            ],
            capture_output=True,
            text=True,
            check=False,
        )
    except OSError:
        # vswhere present but not executable on this host.
        return None

    if completed.returncode != 0:
        return None

    install_path = completed.stdout.strip()
    if not install_path:
        return None

    vcvars = Path(install_path) / "VC" / "Auxiliary" / "Build" / "vcvars64.bat"
    return vcvars if vcvars.is_file() else None


# ---------------------------------------------------------------------------
# Build + dump (default / --fixture path only).
# ---------------------------------------------------------------------------


def _build_dump(vcvars: Path) -> int:
    """Configure and build the ``ctps_dump`` target via vcvars + cmake + ninja.

    Runs a single ``cmd /c`` that sources ``vcvars64.bat`` (to put the MSVC
    toolset, ninja, and cmake on PATH) and then chains the configure and build
    with ``&&`` so a configure failure short-circuits the build. Build output
    streams to this process's stdout/stderr.

    :param vcvars: resolved path to ``vcvars64.bat``.
    :return: the process exit code (``0`` on success).
    """
    _BUILD_DIR.mkdir(parents=True, exist_ok=True)

    # Each path is quoted and ``&&`` chains the steps in one cmd line. This is
    # run via ``shell=True`` (NOT a ["cmd","/c",inner] list): on Windows the
    # list form sends inner through subprocess.list2cmdline, which backslash-
    # escapes the inner quotes (``\"vcvars\"``) so cmd cannot find the batch
    # file. ``shell=True`` instead executes ``cmd /c "<inner>"`` verbatim, the
    # canonical form where cmd strips the outer pair and keeps the inner quotes.
    inner = (
        f'"{vcvars}"'
        f' && cmake -S "{_TEST_DIR}" -B "{_BUILD_DIR}" -G Ninja'
        f' && cmake --build "{_BUILD_DIR}" --target {_DUMP_TARGET}'
    )

    completed = subprocess.run(
        inner,
        cwd=str(_REPO_ROOT),
        shell=True,
        check=False,
    )
    return completed.returncode


def _run_dump(fixture: bool) -> str:
    """Run the built ``ctps_dump`` executable and capture its JSON stdout.

    :param fixture: when ``True``, pass ``--fixture`` so the harness dumps the
        synthetic fixture mission instead of the real ``buildMission``.
    :return: the captured stdout (the Data-Model-A command-stream JSON).
    :raises RuntimeError: if the executable is missing or exits non-zero; the
        message carries the exe's stderr so the caller can surface it verbatim.
    """
    if not _DUMP_EXE.is_file():
        raise RuntimeError(
            f"ctps: build reported success but the dump executable is missing "
            f"at {_DUMP_EXE}."
        )

    argv: List[str] = [str(_DUMP_EXE)]
    if fixture:
        argv.append("--fixture")

    completed = subprocess.run(
        argv,
        cwd=str(_REPO_ROOT),
        capture_output=True,
        text=True,
        check=False,
    )

    if completed.returncode != 0:
        stderr = completed.stderr.strip() or "(no stderr)"
        raise RuntimeError(
            f"ctps: {_DUMP_EXE.name} exited with code {completed.returncode}.\n"
            f"{stderr}"
        )

    return completed.stdout


def _produce_trace_json(fixture: bool) -> Optional[str]:
    """Locate the toolchain, build ``ctps_dump``, and return its JSON stdout.

    This is the default / ``--fixture`` path. On any failure it prints a clear,
    actionable error (always including the ``--trace`` fallback) and returns
    ``None`` so :func:`main` can exit non-zero without raising.

    :param fixture: forward the ``--fixture`` flag to the dump executable.
    :return: the dump JSON string, or ``None`` if the toolchain is missing or
        the build / dump failed.
    """
    vcvars = _find_vcvars()
    if vcvars is None:
        print(
            "ctps: no host C++ toolchain found.\n"
            "  Could not locate vswhere.exe or vcvars64.bat for a Visual "
            "Studio install.\n"
            f"  Looked for vswhere at: {_VSWHERE}\n"
            f"  {_toolchain_hint()}",
            file=sys.stderr,
        )
        return None

    print(f"ctps: building {_DUMP_TARGET} ...", file=sys.stderr)
    build_rc = _build_dump(vcvars)
    if build_rc != 0:
        print(
            f"ctps: cmake configure/build of {_DUMP_TARGET} failed "
            f"(exit {build_rc}). See the build output above.\n"
            f"  {_toolchain_hint()}",
            file=sys.stderr,
        )
        return None

    mission = "fixture mission" if fixture else "real buildMission"
    print(f"ctps: running {_DUMP_EXE.name} ({mission}) ...", file=sys.stderr)
    try:
        return _run_dump(fixture)
    except RuntimeError as exc:
        print(f"{exc}\n  {_toolchain_hint()}", file=sys.stderr)
        return None


# ---------------------------------------------------------------------------
# Render + open (shared by every path).
# ---------------------------------------------------------------------------


def _resolve_map_png() -> Optional[Path]:
    """Return the absolute path to ``field_data.MAP_PNG``, or ``None``.

    Mirrors the visualizer's resolution: a non-absolute ``MAP_PNG`` (e.g.
    ``"maps/map.png"``) is taken relative to :data:`_PLANNER_ROOT`. Returns
    ``None`` when unset or empty so the player renders without a map. A path
    that does not exist is returned as-is; :func:`player_html.render` warns and
    falls back to the vector field.

    :return: the resolved map path, or ``None`` if ``MAP_PNG`` is not set.
    """
    raw = getattr(field_data, "MAP_PNG", None)
    if not raw:
        return None
    p = Path(raw)
    if not p.is_absolute():
        p = _PLANNER_ROOT / p
    return p


def _render_and_open(trace_json: str, out_path: Path, open_browser: bool) -> int:
    """Load -> build -> render the trace JSON, then optionally open the page.

    :param trace_json: a Data-Model-A command-stream JSON document (string).
    :param out_path: destination ``.html`` path for the rendered player.
    :param open_browser: when ``True``, open the rendered page in the default
        browser (``os.startfile`` on Windows, else ``webbrowser.open``).
    :return: ``0`` on success; ``1`` if the JSON is empty/invalid or rendering
        fails.
    """
    if not trace_json.strip():
        print(
            "ctps: the dump produced no JSON (empty output) — nothing to "
            "render.",
            file=sys.stderr,
        )
        return 1

    try:
        trace = trace_loader.load(trace_json)
    except ValueError as exc:
        # Schema mismatch / not-a-dict / bad commands list.
        print(f"ctps: invalid trace JSON: {exc}", file=sys.stderr)
        return 1
    except Exception as exc:  # noqa: BLE001 - json.JSONDecodeError + guards
        print(
            f"ctps: could not parse trace JSON ({type(exc).__name__}: {exc}).",
            file=sys.stderr,
        )
        return 1

    try:
        track = track_builder.build(trace, field_data)
    except KeyError as exc:
        # step_through raises KeyError when a goto targets a node that is not
        # in the LIVE field_data.NODES at all (distinct from UnreachableGoalError,
        # which the builder catches and renders). This is the expected outcome
        # when a trace was recorded against a different field (e.g. a toy-field
        # fixture, or a mission referencing nodes absent from the placeholder
        # field). Report it clearly rather than crashing with a traceback.
        print(
            f"ctps: trace references a node not in the live field_data.NODES "
            f"({exc}).\n"
            "  The trace was recorded against a different field. Regenerate "
            "the dump against the current field, or use a fixture whose nodes "
            "exist in field_data.",
            file=sys.stderr,
        )
        return 1

    out = player_html.render(track, out_path, map_png=_resolve_map_png())

    if open_browser:
        _open_in_browser(out)

    print(str(out))
    return 0


def _open_in_browser(path: Path) -> None:
    """Open ``path`` in the user's default browser, cross-platform.

    Uses ``os.startfile`` on Windows (the most reliable association handler)
    and ``webbrowser.open`` on other platforms via a ``file://`` URI. Any
    failure is reported as a warning and swallowed — the file was already
    written and its path is printed, so a browser miss is non-fatal.

    :param path: the resolved HTML file to open.
    """
    try:
        if sys.platform.startswith("win") and hasattr(os, "startfile"):
            os.startfile(str(path))  # type: ignore[attr-defined] # noqa: S606
        else:
            webbrowser.open(path.as_uri())
    except OSError as exc:
        print(
            f"ctps: could not open a browser ({exc}); the page is at {path}.",
            file=sys.stderr,
        )


# ---------------------------------------------------------------------------
# CLI entry point.
# ---------------------------------------------------------------------------


def main(argv: Optional[List[str]] = None) -> int:
    """Argparse entry point for the CTPS orchestrator.

    :param argv: optional explicit argument list (excluding the program name).
        When ``None`` :mod:`argparse` reads from :data:`sys.argv`.
    :return: process exit code — ``0`` on success, non-zero on any failure
        (missing toolchain, build/dump failure, empty/invalid JSON).
    """
    parser = argparse.ArgumentParser(
        prog="python -m tools.path_planner.ctps",
        description=(
            "Build, dump, and render the CTPS trajectory player in one "
            "command."
        ),
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=_DEFAULT_OUT,
        help=(
            "Output path for the rendered HTML player "
            "(default: tools/path_planner/output/ctps.html)."
        ),
    )
    parser.add_argument(
        "--no-open",
        action="store_true",
        help="Render only; do not open the page in a browser.",
    )
    parser.add_argument(
        "--trace",
        type=Path,
        default=None,
        help=(
            "Skip the C++ build/dump and load this existing dump JSON instead. "
            "This path needs no compiler."
        ),
    )
    parser.add_argument(
        "--fixture",
        action="store_true",
        help=(
            "Dump the synthetic fixture mission (ctps_dump --fixture) instead "
            "of the real buildMission. Ignored when --trace is given."
        ),
    )
    args = parser.parse_args(argv)

    open_browser = not args.no_open

    # --- --trace flow: no compiler ever required -----------------------------
    if args.trace is not None:
        trace_path: Path = args.trace
        if not trace_path.is_file():
            print(
                f"ctps: --trace file not found: {trace_path}", file=sys.stderr
            )
            return 1
        trace_json = trace_path.read_text(encoding="utf-8")
        return _render_and_open(trace_json, args.out, open_browser)

    # --- default / --fixture flow: locate toolchain, build, dump -------------
    trace_json = _produce_trace_json(args.fixture)
    if trace_json is None:
        return 1
    return _render_and_open(trace_json, args.out, open_browser)


if __name__ == "__main__":
    sys.exit(main())
