"""Hermetic unit tests for the CTPS orchestrator's cross-platform branching.

Exercises the platform-specific seams in
:mod:`tools.path_planner.ctps.__main__`:

* :func:`_dump_exe_name` — the ``.exe`` vs bare-name choice keyed on
  ``sys.platform``.
* :func:`_find_dump_exe` — generator-agnostic discovery of the built binary
  across the build root, ``Debug/``, and ``Release/``.
* :func:`_produce_trace_json` — the Windows (vcvars + ninja) vs non-Windows
  (plain cmake) build branch, including the "cmake not found"
  (:class:`FileNotFoundError`) and non-zero-exit (build-failed) error paths.
* :func:`main` — the ``--trace`` flow never touches the compiler.

No real compiler, cmake, or subprocess is invoked: ``sys.platform`` is faked
via ``monkeypatch.setattr(sys, "platform", ...)`` and every build / dump helper
is replaced with an in-test stub. All monkeypatches auto-undo, so the tests are
deterministic and isolated regardless of the host OS.

Covered cases (T8): TC1-TC9 and TC21; TC22-TC24 cover the build-dir self-heal
on a CMake generator mismatch.
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

from tools.path_planner.ctps import __main__ as ctps_main


# ---------------------------------------------------------------------------
# TC1 / TC2 — _dump_exe_name() is keyed on sys.platform
# ---------------------------------------------------------------------------


def test_tc1_dump_exe_name_windows(monkeypatch):
    """TC1: ``_dump_exe_name()`` -> ``"ctps_dump.exe"`` on Windows.

    The SUT keys on ``sys.platform.startswith("win")``; the canonical Windows
    value is ``"win32"`` (which satisfies that prefix check).
    """
    monkeypatch.setattr(sys, "platform", "win32")
    assert ctps_main._dump_exe_name() == "ctps_dump.exe"


def test_tc2_dump_exe_name_non_windows(monkeypatch):
    """TC2: ``_dump_exe_name()`` -> ``"ctps_dump"`` on macOS and Linux."""
    monkeypatch.setattr(sys, "platform", "darwin")
    assert ctps_main._dump_exe_name() == "ctps_dump"

    monkeypatch.setattr(sys, "platform", "linux")
    assert ctps_main._dump_exe_name() == "ctps_dump"


# ---------------------------------------------------------------------------
# TC3 / TC4 / TC5 — _find_dump_exe() is generator-agnostic
# ---------------------------------------------------------------------------


def test_tc3_find_dump_exe_build_root(monkeypatch, tmp_path):
    """TC3: a binary at the build root is found.

    Single-config generators (Ninja, Unix Makefiles) drop the exe directly in
    the build dir; ``_find_dump_exe`` must return that path.
    """
    monkeypatch.setattr(sys, "platform", "linux")  # name -> "ctps_dump"
    monkeypatch.setattr(ctps_main, "_BUILD_DIR", tmp_path)

    exe = tmp_path / "ctps_dump"
    exe.write_text("binary", encoding="utf-8")

    assert ctps_main._find_dump_exe() == exe


def test_tc4_find_dump_exe_debug_subdir(monkeypatch, tmp_path):
    """TC4: a binary that exists ONLY under ``Debug/`` is found.

    Multi-config generators (Visual Studio) nest the exe under ``Debug/`` or
    ``Release/``; with nothing at the build root, the nested binary must win.
    """
    monkeypatch.setattr(sys, "platform", "linux")  # name -> "ctps_dump"
    monkeypatch.setattr(ctps_main, "_BUILD_DIR", tmp_path)

    debug_dir = tmp_path / "Debug"
    debug_dir.mkdir()
    exe = debug_dir / "ctps_dump"
    exe.write_text("binary", encoding="utf-8")

    assert ctps_main._find_dump_exe() == exe


def test_tc5_find_dump_exe_none_when_missing(monkeypatch, tmp_path):
    """TC5: ``_find_dump_exe()`` returns ``None`` when no binary exists.

    The empty (freshly created) build dir holds no candidate in any of the
    three searched locations.
    """
    monkeypatch.setattr(sys, "platform", "linux")
    monkeypatch.setattr(ctps_main, "_BUILD_DIR", tmp_path)

    assert ctps_main._find_dump_exe() is None


# ---------------------------------------------------------------------------
# TC6 — non-Windows build branch: vcvars is NEVER consulted
# ---------------------------------------------------------------------------


def test_tc6_non_windows_branch_skips_vcvars(monkeypatch):
    """TC6: on Linux, the POSIX build branch runs and ``_find_vcvars`` is unused.

    ``_produce_trace_json`` must take the non-Windows path: build via
    ``_build_dump_posix`` (stubbed to succeed), run the dump (stubbed to return
    JSON), and never touch the Windows-only ``_find_vcvars`` discovery.
    """
    monkeypatch.setattr(sys, "platform", "linux")

    vcvars_called = {"hit": False}

    def _fail_find_vcvars():
        vcvars_called["hit"] = True
        raise AssertionError("_find_vcvars must not be called on non-Windows")

    monkeypatch.setattr(ctps_main, "_find_vcvars", _fail_find_vcvars)
    monkeypatch.setattr(ctps_main, "_build_dump_posix", lambda: 0)
    # Guard: the Windows builder must not run on this branch either.
    monkeypatch.setattr(
        ctps_main,
        "_build_dump_windows",
        lambda vcvars: (_ for _ in ()).throw(
            AssertionError("_build_dump_windows must not run on non-Windows")
        ),
    )
    monkeypatch.setattr(ctps_main, "_run_dump", lambda fixture: "JSON")

    result = ctps_main._produce_trace_json(fixture=True)

    assert result == "JSON"
    assert vcvars_called["hit"] is False


# ---------------------------------------------------------------------------
# TC7 — non-Windows: missing cmake (FileNotFoundError) is handled, not raised
# ---------------------------------------------------------------------------


def test_tc7_non_windows_cmake_not_found(monkeypatch, capsys):
    """TC7: a ``FileNotFoundError`` from the POSIX build -> ``None`` + message.

    Models cmake being absent from ``PATH``. ``_produce_trace_json`` must catch
    the error, print a "cmake not found" guidance line to stderr, and return
    ``None`` WITHOUT propagating the exception.
    """
    monkeypatch.setattr(sys, "platform", "linux")
    monkeypatch.setattr(
        ctps_main, "_find_vcvars", _never_called("_find_vcvars")
    )

    def _raise_fnf():
        raise FileNotFoundError("cmake")

    monkeypatch.setattr(ctps_main, "_build_dump_posix", _raise_fnf)
    # If the dump somehow ran, the test would wrongly pass — guard it.
    monkeypatch.setattr(
        ctps_main, "_run_dump", _never_called("_run_dump")
    )

    result = ctps_main._produce_trace_json(fixture=False)

    assert result is None
    err = capsys.readouterr().err
    assert "cmake not found" in err


# ---------------------------------------------------------------------------
# TC8 — Windows build branch: vcvars located, _build_dump_windows is the builder
# ---------------------------------------------------------------------------


def test_tc8_windows_branch_uses_windows_builder(monkeypatch):
    """TC8: on Windows, ``_build_dump_windows`` builds (not the POSIX builder).

    ``_find_vcvars`` returns a fake path; ``_build_dump_windows`` is stubbed to
    succeed and asserts it received that path; ``_build_dump_posix`` must never
    run; the dump returns JSON which ``_produce_trace_json`` passes through.
    """
    monkeypatch.setattr(sys, "platform", "win32")

    fake_vcvars = Path("C:/fake/VC/Auxiliary/Build/vcvars64.bat")
    monkeypatch.setattr(ctps_main, "_find_vcvars", lambda: fake_vcvars)

    windows_called = {"hit": False, "vcvars": None}

    def _stub_build_windows(vcvars):
        windows_called["hit"] = True
        windows_called["vcvars"] = vcvars
        return 0

    monkeypatch.setattr(ctps_main, "_build_dump_windows", _stub_build_windows)
    monkeypatch.setattr(
        ctps_main, "_build_dump_posix", _never_called("_build_dump_posix")
    )
    monkeypatch.setattr(ctps_main, "_run_dump", lambda fixture: "JSON")

    result = ctps_main._produce_trace_json(fixture=False)

    assert result == "JSON"
    assert windows_called["hit"] is True
    assert windows_called["vcvars"] == fake_vcvars


# ---------------------------------------------------------------------------
# TC9 — --trace flow needs no compiler
# ---------------------------------------------------------------------------


def test_tc9_trace_flow_skips_compiler(monkeypatch, tmp_path):
    """TC9: ``main(["--trace", FILE, "--no-open"])`` never builds.

    The ``--trace`` path loads an existing dump and renders it. It must skip
    ``_produce_trace_json`` entirely (stubbed to fail if called) and delegate to
    ``_render_and_open`` (stubbed to return ``0``).
    """
    monkeypatch.setattr(
        ctps_main,
        "_produce_trace_json",
        _never_called("_produce_trace_json"),
    )
    monkeypatch.setattr(
        ctps_main,
        "_render_and_open",
        lambda trace_json, out_path, open_browser, filename="", objects=None: 0,
    )

    trace_file = tmp_path / "run.json"
    trace_file.write_text('{"commands": []}', encoding="utf-8")

    rc = ctps_main.main(["--trace", str(trace_file), "--no-open"])

    assert rc == 0


# ---------------------------------------------------------------------------
# TC21 — non-Windows: build present but FAILS (non-zero exit)
# ---------------------------------------------------------------------------


def test_tc21_non_windows_build_failure_nonzero_exit(monkeypatch, capsys):
    """TC21: a non-zero POSIX build exit -> ``None`` + "failed" message.

    Distinct from TC7's ``FileNotFoundError`` (cmake absent): here cmake is
    present and runs, but the configure/build returns a non-zero code.
    ``_produce_trace_json`` must report the failure to stderr, NOT run the dump,
    and return ``None`` without raising.
    """
    monkeypatch.setattr(sys, "platform", "linux")
    monkeypatch.setattr(
        ctps_main, "_find_vcvars", _never_called("_find_vcvars")
    )
    monkeypatch.setattr(ctps_main, "_build_dump_posix", lambda: 2)
    monkeypatch.setattr(
        ctps_main, "_run_dump", _never_called("_run_dump")
    )

    result = ctps_main._produce_trace_json(fixture=True)

    assert result is None
    err = capsys.readouterr().err
    assert "failed" in err


# ---------------------------------------------------------------------------
# TC22-TC24 — build-dir self-heal on a CMake generator mismatch
# ---------------------------------------------------------------------------


def _write_cache(build_dir: Path, generator: str) -> None:
    """Write a minimal CMakeCache.txt recording ``generator`` into ``build_dir``."""
    build_dir.mkdir(parents=True, exist_ok=True)
    (build_dir / "CMakeCache.txt").write_text(
        "# This is the CMakeCache file.\n"
        f"CMAKE_GENERATOR:INTERNAL={generator}\n",
        encoding="utf-8",
    )


def test_tc22_cached_generator_reads_or_none(monkeypatch, tmp_path):
    """TC22: ``_cached_generator`` returns the cached name, or None when absent.

    No CMakeCache.txt -> None (never configured); a cache carrying the
    ``CMAKE_GENERATOR:INTERNAL=`` line -> that value.
    """
    monkeypatch.setattr(ctps_main, "_BUILD_DIR", tmp_path)
    assert ctps_main._cached_generator() is None

    _write_cache(tmp_path, "Visual Studio 17 2022")
    assert ctps_main._cached_generator() == "Visual Studio 17 2022"


def test_tc23_clear_build_dir_wipes_on_mismatch(monkeypatch, tmp_path):
    """TC23: a non-Ninja cache is wiped so the Ninja reconfigure starts clean.

    Models the reported failure: ``build/ctps`` was configured with the Visual
    Studio generator, but the orchestrator requests Ninja. The stale tree (cache
    + a sentinel VS artifact) must be removed.
    """
    build_dir = tmp_path / "ctps"
    _write_cache(build_dir, "Visual Studio 17 2022")
    (build_dir / "ALL_BUILD.vcxproj").write_text("stale", encoding="utf-8")
    monkeypatch.setattr(ctps_main, "_BUILD_DIR", build_dir)

    ctps_main._clear_build_dir_if_generator_differs("Ninja")

    assert not build_dir.exists()


def test_tc24_clear_build_dir_keeps_matching_or_missing(monkeypatch, tmp_path):
    """TC24: a matching cache (or no cache at all) is left untouched.

    Ninja-on-Ninja must NOT wipe (that would throw away a good incremental
    tree); an absent cache is a no-op (nothing to reconfigure against yet).
    """
    build_dir = tmp_path / "ctps"
    _write_cache(build_dir, "Ninja")
    monkeypatch.setattr(ctps_main, "_BUILD_DIR", build_dir)

    ctps_main._clear_build_dir_if_generator_differs("Ninja")
    assert (build_dir / "CMakeCache.txt").is_file()  # matching -> kept

    empty = tmp_path / "empty"
    empty.mkdir()
    monkeypatch.setattr(ctps_main, "_BUILD_DIR", empty)
    ctps_main._clear_build_dir_if_generator_differs("Ninja")  # no cache -> no-op
    assert empty.exists()


# ---------------------------------------------------------------------------
# Test helpers
# ---------------------------------------------------------------------------


def _never_called(name: str):
    """Return a stub that fails the test if invoked.

    Used to assert a code path is NOT taken: substitute the helper that should
    stay dormant, and any call raises :class:`AssertionError` naming it.

    :param name: the helper name, surfaced in the failure message.
    :return: a function accepting any args that always raises.
    """

    def _stub(*_args, **_kwargs):
        raise AssertionError(f"{name} must not be called on this path")

    return _stub


# ---------------------------------------------------------------------------
# Layout auto-detect — newest tools/path_planner/layout*.json wins
# ---------------------------------------------------------------------------


def test_find_default_layout_none_when_absent(monkeypatch, tmp_path):
    """``_find_default_layout`` -> ``None`` for an empty OR missing layouts dir."""
    # Empty but present dir.
    monkeypatch.setattr(ctps_main, "_LAYOUT_DIR", tmp_path)
    assert ctps_main._find_default_layout() is None

    # Missing dir (the is_dir() guard) — never raises.
    monkeypatch.setattr(ctps_main, "_LAYOUT_DIR", tmp_path / "nope")
    assert ctps_main._find_default_layout() is None


def test_find_default_layout_picks_newest(monkeypatch, tmp_path):
    """``_find_default_layout`` returns the most-recently-modified ``layout*.json``.

    Chrome suffixes repeat editor saves as ``layout (1).json`` etc., so the
    newest file (by mtime) must win regardless of name — the latest Save is the
    one embedded. Explicit mtimes avoid same-tick ties.
    """
    monkeypatch.setattr(ctps_main, "_LAYOUT_DIR", tmp_path)
    canonical = tmp_path / "layout.json"
    suffixed = tmp_path / "layout (1).json"
    canonical.write_text("{}", encoding="utf-8")
    suffixed.write_text("{}", encoding="utf-8")

    os.utime(canonical, (1_000_000, 1_000_000))
    os.utime(suffixed, (2_000_000, 2_000_000))
    assert ctps_main._find_default_layout() == suffixed

    # A fresh Save of the canonical name (now newest) wins back.
    os.utime(canonical, (3_000_000, 3_000_000))
    assert ctps_main._find_default_layout() == canonical


# ---------------------------------------------------------------------------
# --editor flow — emit the authoring page, skip the build/render entirely
# ---------------------------------------------------------------------------


def test_editor_flow_serves_editor_not_player(monkeypatch, tmp_path):
    """``main(["--editor"])`` renders + serves the editor, never the player.

    The ``--editor`` front door must short-circuit before any trace build or
    player render: ``_produce_trace_json`` and ``_render_and_open`` stay dormant.
    ``layout_editor_html.render`` is called once and its bytes handed to
    ``edit_server.serve_editor`` (stubbed so the test binds no socket / never
    blocks).
    """
    calls = {"render": 0, "serve": 0, "layout": "unset"}
    editor_file = tmp_path / "layout_editor.html"

    def _stub_render(field_data, out_path, map_png=None, layout=None):
        calls["render"] += 1
        calls["layout"] = layout
        editor_file.write_bytes(b"<html>editor</html>")
        return editor_file

    def _stub_serve(html_bytes, layout_out, open_browser=True, host="127.0.0.1"):
        calls["serve"] += 1
        assert html_bytes == b"<html>editor</html>"
        assert Path(layout_out) == ctps_main._DEFAULT_LAYOUT
        return 0

    monkeypatch.setattr(ctps_main.layout_editor_html, "render", _stub_render)
    monkeypatch.setattr(ctps_main.edit_server, "serve_editor", _stub_serve)
    # Isolate from the real layouts/ dir so this test is deterministic.
    monkeypatch.setattr(ctps_main, "_find_default_layout", lambda: None)
    monkeypatch.setattr(
        ctps_main, "_produce_trace_json", _never_called("_produce_trace_json")
    )
    monkeypatch.setattr(
        ctps_main, "_render_and_open", _never_called("_render_and_open")
    )

    rc = ctps_main.main(["--editor", "--no-open"])

    assert rc == 0
    assert calls["render"] == 1 and calls["serve"] == 1
    # No layout present -> render() is asked to open an empty editor.
    assert calls["layout"] is None


def test_editor_flow_autoloads_recent_layout(monkeypatch, tmp_path):
    """``main(["--editor"])`` auto-loads the recently-saved layout into render().

    The newest ``layouts/layout*.json`` (:func:`_find_default_layout`) is read
    and passed to ``layout_editor_html.render(..., layout=...)`` so the editor
    opens on the last layout rather than a fresh scene.
    """
    layout_dict = {
        "schema_version": 1,
        "objects": [
            {
                "id": "o1", "name": "block", "shape": "rect", "type": "cargo",
                "color": "#5bc8e0", "x_cm": 1.0, "y_cm": 2.0,
                "heading_deg": 0.0, "w_cm": 5.0, "h_cm": 5.0,
            }
        ],
        "robot": {
            "grab_zone": {"x_cm": 0.0, "y_cm": 0.0, "w_cm": 6.0, "h_cm": 6.0},
            "box_zone": {"x_cm": -9.0, "y_cm": 0.0, "w_cm": 6.0, "h_cm": 6.0},
            "drop_points": [{"x_cm": 4.0, "y_cm": -6.0}],
        },
    }
    layout_path = tmp_path / "layout.json"
    layout_path.write_text(json.dumps(layout_dict), encoding="utf-8")
    monkeypatch.setattr(ctps_main, "_find_default_layout", lambda: layout_path)

    captured = {}

    def _stub_render(field_data, out_path, map_png=None, layout=None):
        captured["layout"] = layout
        out = tmp_path / "layout_editor.html"
        out.write_bytes(b"<html>editor</html>")
        return out

    monkeypatch.setattr(ctps_main.layout_editor_html, "render", _stub_render)
    monkeypatch.setattr(ctps_main.edit_server, "serve_editor", lambda *a, **k: 0)

    rc = ctps_main.main(["--editor", "--no-open"])

    assert rc == 0
    assert captured["layout"] == layout_dict


def test_editor_flow_unreadable_layout_opens_empty(monkeypatch, tmp_path):
    """A corrupt recent layout degrades to an empty editor, never a hard fail."""
    bad = tmp_path / "layout.json"
    bad.write_text("{ not valid json", encoding="utf-8")
    monkeypatch.setattr(ctps_main, "_find_default_layout", lambda: bad)

    captured = {}

    def _stub_render(field_data, out_path, map_png=None, layout=None):
        captured["layout"] = layout
        out = tmp_path / "layout_editor.html"
        out.write_bytes(b"<html>editor</html>")
        return out

    monkeypatch.setattr(ctps_main.layout_editor_html, "render", _stub_render)
    monkeypatch.setattr(ctps_main.edit_server, "serve_editor", lambda *a, **k: 0)

    rc = ctps_main.main(["--editor", "--no-open"])

    assert rc == 0
    assert captured["layout"] is None
