"""CTPS one-command orchestrator — build, dump, render, open.

Run as a module from the repo root::

    python -m tools.path_planner.ctps
    python -m tools.path_planner.ctps --fixture
    python -m tools.path_planner.ctps --trace run.json --no-open
    python -m tools.path_planner.ctps --fixture --layout my_layout.json
    python -m tools.path_planner.ctps --editor   # open the layout editor

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
   When a cargo layout is given via ``--layout FILE`` (or the conventional
   ``tools/path_planner/layout.json`` exists), it is embedded so the player
   auto-loads its objects/zones on open — no in-browser upload.
#. Open the page in the default browser (unless ``--no-open``) and print the
   output path.

Toolchain policy (CTPS plan, Error Handling): the Python side NEVER requires a
C++ compiler when ``--trace`` is supplied. The compiler is located lazily, only
on the default / ``--fixture`` build path, and the build is platform-specific:

* On Windows the MSVC toolset is located via ``vswhere.exe`` ->
  ``vcvars64.bat`` and the dump is built with cmake + Ninja inside a sourced
  ``vcvars`` shell.
* On non-Windows hosts there is no vcvars step: the dump is built with plain
  ``cmake`` using the default generator and whatever C++ compiler (g++/clang)
  is already on ``PATH`` — mirroring the CI invocation.

A missing toolchain is reported with a clear message that points the user at
``--trace <existing_dump.json>`` and exits non-zero.

Stdlib only — no third-party dependencies.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import webbrowser
from pathlib import Path
from typing import Any, Dict, List, Optional

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
from tools.path_planner.ctps import (  # noqa: E402
    edit_server,
    layout_editor_html,
    player_html,
    track_builder,
    trace_loader,
)
from tools.path_planner.ctps.layout import LayoutError  # noqa: E402

# ---------------------------------------------------------------------------
# Constants — paths and the C++ harness wiring.
# ---------------------------------------------------------------------------

#: Default output path for the rendered HTML player, relative to the repo root.
_DEFAULT_OUT: Path = _REPO_ROOT / "tools" / "path_planner" / "output" / "ctps.html"

#: Output path for the offline layout editor page (`--editor`), beside the player.
_EDITOR_OUT: Path = _DEFAULT_OUT.parent / "layout_editor.html"

#: path_planner root. ``field_data.MAP_PNG`` (e.g. "maps/map.png") resolves
#: relative to this, matching the visualizer's map resolution so the player's
#: embedded background and the static map render agree.
_PLANNER_ROOT: Path = _REPO_ROOT / "tools" / "path_planner"

#: Dedicated home for author-saved cargo layouts (its own folder, not loose in
#: the pipeline dir; see ``layouts/README.md``). The offline editor's Save
#: downloads a ``layout.json`` — point the browser's Save dialog here once and
#: :func:`_find_default_layout` embeds the newest match on the next render, no
#: ``--layout`` flag and no in-browser upload.
_LAYOUT_DIR: Path = _PLANNER_ROOT / "layouts"

#: Canonical name of the auto-embedded cargo layout (the editor's Save downloads
#: exactly this name, into :data:`_LAYOUT_DIR`).
_DEFAULT_LAYOUT: Path = _LAYOUT_DIR / "layout.json"

#: Glob for the working layout(s): Chrome suffixes repeat downloads of the same
#: name as ``layout (1).json`` etc., so the auto-detect matches all of them and
#: picks the newest (see :func:`_find_default_layout`).
_LAYOUT_GLOB: str = "layout*.json"

#: CMake source dir for the host test harness (holds the ``ctps_dump`` target).
_TEST_DIR: Path = _REPO_ROOT / "WRO2026" / "test"

#: Out-of-tree build dir for the ``ctps_dump`` target (gitignored).
_BUILD_DIR: Path = _REPO_ROOT / "build" / "ctps"

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
# Built-binary discovery (generator-agnostic, cross-platform).
# ---------------------------------------------------------------------------


def _dump_exe_name() -> str:
    """Return the platform-specific file name of the built dump executable.

    :return: ``"ctps_dump.exe"`` on Windows, ``"ctps_dump"`` elsewhere.
    """
    return "ctps_dump.exe" if sys.platform.startswith("win") else "ctps_dump"


def _find_dump_exe() -> Optional[Path]:
    """Locate the freshly built ``ctps_dump`` binary, generator-agnostic.

    Different CMake generators drop the binary in different places: single-
    config generators (Ninja, Unix Makefiles) put it at the build root, while
    multi-config generators (Visual Studio) nest it under ``Debug/`` or
    ``Release/``. This searches all three locations for the platform exe name
    and returns the newest existing match so a stale binary from a previous
    generator never shadows the one just built.

    :return: the most-recently-modified matching executable, or ``None`` when
        no build output exists yet.
    """
    name = _dump_exe_name()
    candidates: List[Path] = [
        _BUILD_DIR / name,
        _BUILD_DIR / "Debug" / name,
        _BUILD_DIR / "Release" / name,
    ]
    existing = [c for c in candidates if c.is_file()]
    if not existing:
        return None
    return max(existing, key=lambda p: p.stat().st_mtime)


# ---------------------------------------------------------------------------
# Toolchain discovery (Windows default / --fixture path only).
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
    """Locate ``vcvars64.bat`` via ``vswhere.exe`` (Windows only).

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
            encoding="utf-8",
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
# Build (default / --fixture path only) — one path per platform.
# ---------------------------------------------------------------------------


def _cached_generator() -> Optional[str]:
    """Return the ``CMAKE_GENERATOR`` recorded in ``_BUILD_DIR``'s cache, or None.

    Reads the ``CMAKE_GENERATOR:INTERNAL=<name>`` line from
    ``_BUILD_DIR/CMakeCache.txt``. Returns ``None`` when the build dir has no
    cache (never configured), the file is unreadable, or the key is absent.

    :return: the cached generator name (e.g. ``"Ninja"`` /
        ``"Visual Studio 17 2022"``), or ``None``.
    """
    cache = _BUILD_DIR / "CMakeCache.txt"
    try:
        text = cache.read_text(encoding="utf-8", errors="replace")
    except OSError:
        # Missing (never configured) or unreadable — treat as "no generator".
        return None
    for line in text.splitlines():
        if line.startswith("CMAKE_GENERATOR:"):
            _key, _sep, value = line.partition("=")
            return value.strip() or None
    return None


def _clear_build_dir_if_generator_differs(expected: str) -> None:
    """Wipe ``_BUILD_DIR`` when its cached generator is not ``expected``.

    CMake refuses to reconfigure an existing build tree with a different
    generator (``Error: generator : Ninja Does not match the generator used
    previously: Visual Studio 17 2022``). That collision happens when something
    else — e.g. a VS Code CMake Tools configure — writes a non-Ninja cache into
    the orchestrator's own ``build/ctps`` dir. Rather than hard-fail with that
    opaque message, detect the mismatch and remove the dir so the fresh
    configure that follows starts clean. A dir with no cache, or one already on
    ``expected``, is left untouched.

    :param expected: the generator the caller is about to request (``"Ninja"``).
    """
    cached = _cached_generator()
    if cached is not None and cached != expected:
        print(
            f"ctps: {_BUILD_DIR} was configured with {cached!r}, not "
            f"{expected!r}; wiping it for a clean reconfigure.",
            file=sys.stderr,
        )
        shutil.rmtree(_BUILD_DIR, ignore_errors=True)


def _build_dump_windows(vcvars: Path) -> int:
    """Configure and build ``ctps_dump`` via vcvars + cmake + ninja (Windows).

    Runs a single ``cmd /c`` that sources ``vcvars64.bat`` (to put the MSVC
    toolset, ninja, and cmake on PATH) and then chains the configure and build
    with ``&&`` so a configure failure short-circuits the build. Build output
    streams to this process's stdout/stderr. Before configuring, a stale build
    tree left by a different generator (see
    :func:`_clear_build_dir_if_generator_differs`) is wiped so the ``-G Ninja``
    configure never hits CMake's generator-mismatch error.

    :param vcvars: resolved path to ``vcvars64.bat``.
    :return: the process exit code (``0`` on success).
    """
    _clear_build_dir_if_generator_differs("Ninja")
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


def _build_dump_posix() -> int:
    """Configure and build ``ctps_dump`` via plain cmake (non-Windows).

    Mirrors the CI invocation: a default-generator configure
    (``cmake -S WRO2026/test -B build/ctps``) followed by
    ``cmake --build build/ctps --target ctps_dump``, using whatever C++
    compiler is already on ``PATH``. No vcvars / no shell — each command is an
    argv list run with ``shell=False`` so paths with spaces need no quoting.
    Build output streams to this process's stdout/stderr.

    A missing ``cmake`` raises :class:`FileNotFoundError`, which is allowed to
    PROPAGATE to the caller so it can emit the "cmake not found" guidance; this
    function does not catch it.

    :return: the configure exit code if configure failed, otherwise the build
        exit code (``0`` on success).
    """
    _BUILD_DIR.mkdir(parents=True, exist_ok=True)

    configure = subprocess.run(
        ["cmake", "-S", str(_TEST_DIR), "-B", str(_BUILD_DIR)],
        cwd=str(_REPO_ROOT),
        check=False,
    )
    if configure.returncode != 0:
        return configure.returncode

    build = subprocess.run(
        ["cmake", "--build", str(_BUILD_DIR), "--target", _DUMP_TARGET],
        cwd=str(_REPO_ROOT),
        check=False,
    )
    return build.returncode


def _run_dump(fixture: bool) -> str:
    """Run the built ``ctps_dump`` executable and capture its JSON stdout.

    The binary is located with :func:`_find_dump_exe` so the lookup is
    generator-agnostic (build root, ``Debug/``, or ``Release/``).

    :param fixture: when ``True``, pass ``--fixture`` so the harness dumps the
        synthetic fixture mission instead of the real ``buildMission``.
    :return: the captured stdout (the Data-Model-A command-stream JSON).
    :raises RuntimeError: if the executable is missing or exits non-zero; the
        message carries the exe's stderr so the caller can surface it verbatim.
    """
    exe = _find_dump_exe()
    if exe is None:
        raise RuntimeError(
            f"ctps: build reported success but the dump executable is missing "
            f"in {_BUILD_DIR}."
        )

    argv: List[str] = [str(exe)]
    if fixture:
        argv.append("--fixture")

    completed = subprocess.run(
        argv,
        cwd=str(_REPO_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        check=False,
    )

    if completed.returncode != 0:
        stderr = completed.stderr.strip() or "(no stderr)"
        raise RuntimeError(
            f"ctps: {exe.name} exited with code {completed.returncode}.\n"
            f"{stderr}"
        )

    return completed.stdout


def _produce_trace_json(fixture: bool) -> Optional[str]:
    """Build ``ctps_dump`` for this platform and return its JSON stdout.

    This is the default / ``--fixture`` path. On Windows the MSVC toolchain is
    located via :func:`_find_vcvars` and the dump is built with vcvars + Ninja;
    on non-Windows hosts vcvars is skipped entirely and the dump is built with
    plain ``cmake`` (default generator, compiler from ``PATH``). On any failure
    it prints a clear, actionable error (always including the ``--trace``
    fallback) and returns ``None`` so :func:`main` can exit non-zero without
    raising.

    :param fixture: forward the ``--fixture`` flag to the dump executable.
    :return: the dump JSON string, or ``None`` if the toolchain is missing or
        the build / dump failed.
    """
    print(f"ctps: building {_DUMP_TARGET} ...", file=sys.stderr)

    if sys.platform.startswith("win"):
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
        build_rc = _build_dump_windows(vcvars)
    else:
        try:
            build_rc = _build_dump_posix()
        except FileNotFoundError:
            print(
                "ctps: cmake not found on PATH. Install CMake and a C++ "
                "compiler (g++/clang).\n"
                f"  {_toolchain_hint()}",
                file=sys.stderr,
            )
            return None

    if build_rc != 0:
        print(
            f"ctps: cmake configure/build of {_DUMP_TARGET} failed "
            f"(exit {build_rc}). See the build output above.\n"
            f"  {_toolchain_hint()}",
            file=sys.stderr,
        )
        return None

    mission = "fixture mission" if fixture else "real buildMission"
    print(f"ctps: running {_dump_exe_name()} ({mission}) ...", file=sys.stderr)
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


def _render_and_open(
    trace_json: str,
    out_path: Path,
    open_browser: bool,
    filename: str = "",
    objects: "Optional[Dict[str, Any]]" = None,
) -> int:
    """Load -> build -> render the trace JSON, then optionally open the page.

    :param trace_json: a Data-Model-A command-stream JSON document (string).
    :param out_path: destination ``.html`` path for the rendered player.
    :param open_browser: when ``True``, open the rendered page in the default
        browser (``os.startfile`` on Windows, else ``webbrowser.open``).
    :param filename: display filename embedded in the page header.
    :param objects: optional ``layout.json``-shaped dict embedded as
        ``const OBJECTS`` so the player auto-loads its cargo layout (no
        in-browser upload); ``None`` renders a layout-free page.
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

    try:
        out = player_html.render(
            track,
            out_path,
            map_png=_resolve_map_png(),
            filename=filename,
            objects=objects,
        )
    except LayoutError as exc:
        # A malformed layout must fail the render loud rather than silently
        # emit a player with no cargo — the layout is caller-supplied, not
        # field-day content.
        print(f"ctps: invalid layout: {exc}", file=sys.stderr)
        return 1

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


def _find_default_layout() -> Optional[Path]:
    """Return the newest ``layout*.json`` in :data:`_LAYOUT_DIR`, or ``None``.

    The offline editor's Save is a browser download named ``layout.json``
    (:data:`_DEFAULT_LAYOUT`); Chrome suffixes repeat saves of the same name as
    ``layout (1).json``, ``layout (2).json``, ... Picking the most-recently-
    modified match keeps the "just click Save" flow frictionless — the latest
    Save always wins — without the author renaming or deleting the older files.
    An explicit ``--layout`` bypasses this entirely. A missing ``layouts/``
    folder is treated as "no layout".

    :return: the newest matching layout file, or ``None`` when none exists.
    """
    if not _LAYOUT_DIR.is_dir():
        return None
    matches = [p for p in _LAYOUT_DIR.glob(_LAYOUT_GLOB) if p.is_file()]
    if not matches:
        return None
    return max(matches, key=lambda p: p.stat().st_mtime)


def _serve_editor(open_browser: bool) -> int:
    """Render the offline layout editor and serve it on a self-closing server.

    The ``--editor`` front door: renders the self-contained ``layout_editor.html``
    (field + robot tabs) against the LIVE :mod:`tools.path_planner.field_data`,
    auto-loading the recently-saved layout (:func:`_find_default_layout`) so it
    opens on the last layout rather than a fresh scene, then hands it to
    :func:`edit_server.serve_editor`, which serves it on a
    localhost port and writes every Save straight into :data:`_DEFAULT_LAYOUT`
    (``layouts/layout.json``) — no download dialog. The server auto-closes when
    the editor tab goes away (heartbeat + watchdog, ADR-057), so it can never
    dangle. Needs no C++ toolchain and no trace. The next
    ``python -m tools.path_planner.ctps`` auto-embeds the saved layout (see
    :func:`_find_default_layout` and ``layouts/README.md``).

    :param open_browser: when ``True``, open the served URL after binding.
    :return: ``0`` on a clean shutdown; ``1`` if rendering fails.
    """
    # Auto-load the recently-saved layout (newest layouts/layout*.json — the
    # same file Save writes) so the editor opens on the last layout instead of a
    # fresh scene. Degrade to an empty editor (never a hard fail) when it is
    # missing or unreadable: the author must always be able to open the tool.
    layout: Optional[Dict[str, Any]] = None
    layout_path = _find_default_layout()
    if layout_path is not None:
        try:
            loaded = json.loads(layout_path.read_text(encoding="utf-8"))
            if isinstance(loaded, dict):
                layout = loaded
                print(
                    f"ctps: editor auto-loading {layout_path.name}",
                    file=sys.stderr,
                )
        except (OSError, ValueError) as exc:
            print(
                f"ctps: could not read {layout_path.name} ({exc}); "
                "opening the editor empty",
                file=sys.stderr,
            )

    try:
        out = layout_editor_html.render(
            field_data, _EDITOR_OUT, map_png=_resolve_map_png(), layout=layout
        )
    except (OSError, ValueError, AttributeError) as exc:
        print(f"ctps: could not render the layout editor: {exc}", file=sys.stderr)
        return 1

    return edit_server.serve_editor(
        out.read_bytes(), _DEFAULT_LAYOUT, open_browser=open_browser
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
    parser.add_argument(
        "--layout",
        type=Path,
        default=None,
        help=(
            "Embed this layout.json (field objects + robot zones, from the "
            "offline layout editor) so the player auto-loads it — no in-browser "
            "'Load layout…' upload. Defaults to the newest "
            "tools/path_planner/layouts/layout*.json when one exists."
        ),
    )
    parser.add_argument(
        "--editor",
        action="store_true",
        help=(
            "Open the offline layout editor (layout_editor.html) instead of the "
            "player — author field objects + robot zones, then Save. Ignores "
            "--trace/--fixture/--layout; needs no C++ toolchain."
        ),
    )
    args = parser.parse_args(argv)

    open_browser = not args.no_open

    # --- --editor flow: serve the authoring page, nothing else ---------------
    if args.editor:
        return _serve_editor(open_browser)

    # Resolve the cargo layout to embed (object/cargo plan, issue #57): the
    # explicit --layout wins; otherwise the newest layouts/layout*.json is
    # auto-embedded when one exists. The emitted player then auto-loads it on
    # open — no in-browser upload. A layout-free run stays byte-clean (no
    # const OBJECTS). Resolved once here so both the --trace and build paths
    # embed it, and a bad --layout fails fast before any C++ build.
    layout_path: Optional[Path] = args.layout
    if layout_path is None:
        layout_path = _find_default_layout()
    layout: Optional[Dict[str, Any]] = None
    if layout_path is not None:
        if not layout_path.is_file():
            print(
                f"ctps: --layout file not found: {layout_path}", file=sys.stderr
            )
            return 1
        try:
            layout = json.loads(layout_path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            print(
                f"ctps: could not read layout {layout_path}: {exc}",
                file=sys.stderr,
            )
            return 1
        if layout:
            print(
                f"ctps: embedding layout {layout_path.name}", file=sys.stderr
            )

    # --- --trace flow: no compiler ever required -----------------------------
    if args.trace is not None:
        trace_path: Path = args.trace
        if not trace_path.is_file():
            print(
                f"ctps: --trace file not found: {trace_path}", file=sys.stderr
            )
            return 1
        trace_json = trace_path.read_text(encoding="utf-8")
        return _render_and_open(
            trace_json,
            args.out,
            open_browser,
            filename=trace_path.name,
            objects=layout,
        )

    # --- default / --fixture flow: locate toolchain, build, dump -------------
    trace_json = _produce_trace_json(args.fixture)
    if trace_json is None:
        return 1
    return _render_and_open(
        trace_json,
        args.out,
        open_browser,
        filename=("fixture mission" if args.fixture else "mission"),
        objects=layout,
    )


if __name__ == "__main__":
    sys.exit(main())
