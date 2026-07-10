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
# TC25-TC27 — routes freshness self-heal before the dump build
# ---------------------------------------------------------------------------


def test_tc25_routes_match_live_field_uses_embedded_source_hash(monkeypatch, tmp_path):
    """TC25: matching ``Source SHA256`` banners report fresh routes.

    CTPS compares the live field-data hash against the generated
    ``routes.h/routes.cpp`` banners rather than mtime heuristics. A matching
    hash in both files returns ``True``.
    """
    from tools.path_planner import header_writer

    routes_dir = tmp_path / "routes"
    routes_dir.mkdir()
    header = routes_dir / "routes.h"
    source = routes_dir / "routes.cpp"
    header.write_text("// routes.h\n// Source SHA256: abc123\n", encoding="utf-8")
    source.write_text("// routes.cpp\n// Source SHA256: abc123\n", encoding="utf-8")

    monkeypatch.setattr(header_writer, "DEFAULT_HEADER_PATH", header.relative_to(tmp_path))
    monkeypatch.setattr(header_writer, "DEFAULT_SOURCE_PATH", source.relative_to(tmp_path))
    monkeypatch.setattr(header_writer, "compute_source_hash", lambda *paths: "abc123")
    monkeypatch.setattr(ctps_main, "_REPO_ROOT", tmp_path)

    assert ctps_main._routes_match_live_field() is True


def test_tc26_ensure_routes_fresh_regenerates_when_stale(monkeypatch, tmp_path):
    """TC26: stale routes trigger ``build_routes._run(no_viz=True)`` once."""
    from tools.path_planner import build_routes

    calls = {"count": 0, "no_viz": None}

    monkeypatch.setattr(ctps_main, "_routes_match_live_field", lambda: False)

    def _stub_run(*, out_dir=None, map_out=None, no_viz=False, verbose=False):
        calls["count"] += 1
        calls["no_viz"] = no_viz
        return 0

    monkeypatch.setattr(build_routes, "_run", _stub_run)

    assert ctps_main._ensure_routes_fresh() is True
    assert calls["count"] == 1
    assert calls["no_viz"] is True


def test_tc27_produce_trace_json_aborts_when_routes_regen_fails(monkeypatch):
    """TC27: a failed routes refresh stops before any compiler/dump work."""
    monkeypatch.setattr(sys, "platform", "linux")
    monkeypatch.setattr(ctps_main, "_ensure_routes_fresh", lambda: False)
    monkeypatch.setattr(ctps_main, "_build_dump_posix", _never_called("_build_dump_posix"))
    monkeypatch.setattr(ctps_main, "_run_dump", _never_called("_run_dump"))

    assert ctps_main._produce_trace_json(fixture=False) is None


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

    def _stub_serve(
        html_bytes, layout_out, open_browser=True, host="127.0.0.1", port=0,
        current_html=None,
    ):
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


def test_editor_flow_threads_fixed_port(monkeypatch, tmp_path):
    """``main(["--editor", "--port", N])`` threads the fixed port to ``serve_editor``.

    The Simple-Browser task opens a hardcoded ``http://127.0.0.1:<port>/``, so the
    port on the CLI must reach the server binder verbatim.
    """
    monkeypatch.setattr(ctps_main, "_find_default_layout", lambda: None)

    captured = {}

    def _stub_render(field_data, out_path, map_png=None, layout=None):
        out = tmp_path / "layout_editor.html"
        out.write_bytes(b"<html>editor</html>")
        return out

    def _stub_serve(html_bytes, layout_out, open_browser=True, host="127.0.0.1", port=0,
                    current_html=None):
        captured["port"] = port
        return 0

    monkeypatch.setattr(ctps_main.layout_editor_html, "render", _stub_render)
    monkeypatch.setattr(ctps_main.edit_server, "serve_editor", _stub_serve)

    rc = ctps_main.main(["--editor", "--no-open", "--port", "8771"])

    assert rc == 0
    assert captured["port"] == 8771


# ---------------------------------------------------------------------------
# --stop-player flow — evict a prior localhost server, build/render nothing
# ---------------------------------------------------------------------------


def test_stop_player_flow_evicts_and_exits(monkeypatch):
    """``main(["--stop-player", "--port", N])`` only POSTs /close, then exits 0.

    The fresh-start task's front door must short-circuit before any build,
    render, or serve: it evicts the incumbent server on the given port via
    ``edit_server._request_close`` and returns 0. Every build/render/serve
    helper stays dormant, so a quote-free ``--stop-player --port N`` argv is a
    pure no-op except for the best-effort close beacon.
    """
    captured = {}

    def _stub_request_close(host, port):
        captured["host"] = host
        captured["port"] = port

    monkeypatch.setattr(
        ctps_main.edit_server, "_request_close", _stub_request_close
    )
    monkeypatch.setattr(
        ctps_main, "_produce_trace_json", _never_called("_produce_trace_json")
    )
    monkeypatch.setattr(
        ctps_main, "_render_and_open", _never_called("_render_and_open")
    )
    monkeypatch.setattr(
        ctps_main, "_render_and_serve", _never_called("_render_and_serve")
    )
    monkeypatch.setattr(
        ctps_main, "_serve_editor", _never_called("_serve_editor")
    )

    rc = ctps_main.main(["--stop-player", "--port", "8770"])

    assert rc == 0
    assert captured == {"host": "127.0.0.1", "port": 8770}


# ---------------------------------------------------------------------------
# --serve flow — render then serve over http instead of writing + opening
# ---------------------------------------------------------------------------


def test_inject_player_heartbeat_before_body_close():
    """``_inject_player_heartbeat`` inserts the liveness script just before </body>."""
    html = "<html><body><div>player</div></body></html>"
    out = ctps_main._inject_player_heartbeat(html)

    # The heartbeat rides inside the body, before its close.
    assert "/ping" in out
    assert "/close" in out
    body_close = out.index("</body>")
    assert out.index("/ping") < body_close
    # Same-origin relative URLs only — never an absolute http(s) literal (so it
    # would survive the player's self-containment lint if it ever moved on-page).
    assert "http://" not in out
    assert "https://" not in out
    # The file:// guard keeps a plain double-clicked player untouched.
    assert 'protocol === "file:"' in out


def test_inject_player_heartbeat_appends_when_no_body():
    """With no </body>, the script is appended rather than lost."""
    out = ctps_main._inject_player_heartbeat("<html>bare</html>")
    assert "/ping" in out
    assert out.rstrip().endswith("</script>")


def test_serve_flow_dispatches_to_serve(monkeypatch, tmp_path):
    """``main(["--serve", "--trace", FILE])`` serves and never opens a file.

    The ``--serve`` path must route through ``_render_and_serve`` (stubbed to
    capture its port) and leave ``_render_and_open`` dormant — the opposite of
    the default flow.
    """
    captured = {}

    def _stub_serve(trace_json, out_path, open_browser, port, filename="", objects=None):
        captured["port"] = port
        captured["open_browser"] = open_browser
        return 0

    monkeypatch.setattr(ctps_main, "_render_and_serve", _stub_serve)
    monkeypatch.setattr(
        ctps_main, "_render_and_open", _never_called("_render_and_open")
    )
    monkeypatch.setattr(
        ctps_main, "_produce_trace_json", _never_called("_produce_trace_json")
    )

    trace_file = tmp_path / "run.json"
    trace_file.write_text('{"commands": []}', encoding="utf-8")

    rc = ctps_main.main(
        ["--serve", "--port", "8770", "--no-open", "--trace", str(trace_file)]
    )

    assert rc == 0
    assert captured["port"] == 8770
    assert captured["open_browser"] is False


# ---------------------------------------------------------------------------
# T14 (issue #130) — the new T8 drive fields / declare_position op introduce
# no NEW platform-sensitive surface.
#
# Rationale (documented here since there is no new branching to test): the
# genuinely platform-sensitive seams this module guards are `sys.platform`
# branching in `ctps/__main__.py` (TC1-TC9, TC21-TC27 above) and file-system
# path handling (`_find_dump_exe`, `_find_default_layout`) — NONE of which T8
# (recorder fields) / T9 (loader parsing) / T10 (densification) / T11
# (return-to-node + declare_position) touch. The new C++-side JSON fields use
# `std::to_string(static_cast<int>(...))` for `power`/`start_speed`/
# `end_speed` (locale-independent, unlike the pre-existing `%g` float
# formatting `cm`/`heading`/`deg` already use) and plain string/bool literals
# for `preset`/`profiled`/`brake`/`node` — no new float-formatting surface.
# `trace_loader.load` reads every fixture with explicit `encoding="utf-8"`
# (Python's text-mode universal-newline translation already normalises
# `\r\n`/`\n`/`\r` transparently), and `player_html.render` writes with the
# existing `newline=""` + UTF-8 discipline (TC5's self-containment lint,
# unchanged by this feature). This test is the explicit confirmation that
# nothing about the new fields depends on the checked-out line-ending style —
# the one plausible cross-platform vector a Windows-vs-POSIX checkout could
# introduce for a hand-authored JSON fixture.
# ---------------------------------------------------------------------------


def test_t14_new_drive_fields_load_identically_regardless_of_line_endings(tmp_path):
    """The T8 fields parse identically whether the fixture uses CRLF or LF.

    Writes the SAME JSON body carrying every new field
    (`power`/`preset`/`start_speed`/`end_speed`/`profiled`/`brake` plus a
    `declare_position` op) twice — once with LF, once with CRLF line endings
    (mirroring the two states a Windows vs. POSIX ``git`` checkout could
    produce for a text fixture) — and asserts ``trace_loader.load`` parses
    both into field-for-field identical :class:`~tools.path_planner.ctps.trace_loader.Command`
    lists. Uses the module's own :mod:`tools.path_planner.ctps.trace_loader`
    (never a hand-rolled parser), so this proves the actual seam, not a
    reimplementation of it.
    """
    from tools.path_planner.ctps import trace_loader

    body_lf = (
        '{\n'
        '  "schema_version": 1,\n'
        '  "start_node": "A",\n'
        '  "start_heading_deg": 0.0,\n'
        '  "build_error": false,\n'
        '  "commands": [\n'
        '    { "seq": 0, "op": "drive", "method": "driveDist", "cm": 12.0,\n'
        '      "power": 90, "preset": "fast", "start_speed": 40, "end_speed": 10,\n'
        '      "profiled": true, "brake": false },\n'
        '    { "seq": 1, "op": "declare_position", "node": "B" }\n'
        '  ]\n'
        '}\n'
    )
    body_crlf = body_lf.replace("\n", "\r\n")
    assert body_lf != body_crlf  # the two fixtures are genuinely different bytes

    path_lf = tmp_path / "lf.json"
    path_crlf = tmp_path / "crlf.json"
    path_lf.write_bytes(body_lf.encode("utf-8"))
    path_crlf.write_bytes(body_crlf.encode("utf-8"))

    trace_lf = trace_loader.load(path_lf)
    trace_crlf = trace_loader.load(path_crlf)

    assert len(trace_lf.commands) == len(trace_crlf.commands) == 2
    for cmd_lf, cmd_crlf in zip(trace_lf.commands, trace_crlf.commands):
        assert cmd_lf == cmd_crlf


def test_t14_full_seam_fixture_renders_identically_regardless_of_line_endings(tmp_path):
    """End to end: the T14 fixture builds + renders the same track under CRLF/LF.

    Reads the COMMITTED ``ctps_full_seam_trace.json`` fixture, re-writes it
    twice (LF-normalised and CRLF), and asserts ``track_builder.build`` (T10/
    T11 densification + localization events) produces byte-identical
    ``Track.to_dict()`` output for both — the same drift-free, event-bearing
    trace this suite's other T14 tests exercise (see
    ``test_ctps_end_to_end.py``), here specifically isolating line-ending
    sensitivity end to end rather than just at the loader.
    """
    import json as json_mod
    import warnings as warnings_mod

    from tools.path_planner.ctps import trace_loader, track_builder
    from tools.path_planner.tests.fixtures import toy_field_data

    fixture_path = (
        Path(__file__).resolve().parent / "fixtures" / "ctps_full_seam_trace.json"
    )
    raw = fixture_path.read_text(encoding="utf-8")
    normalized_lf = raw.replace("\r\n", "\n")
    normalized_crlf = normalized_lf.replace("\n", "\r\n")

    path_lf = tmp_path / "seam_lf.json"
    path_crlf = tmp_path / "seam_crlf.json"
    path_lf.write_bytes(normalized_lf.encode("utf-8"))
    path_crlf.write_bytes(normalized_crlf.encode("utf-8"))

    def _build(path):
        trace = trace_loader.load(path)
        with warnings_mod.catch_warnings():
            warnings_mod.simplefilter("ignore", UserWarning)
            return track_builder.build(trace, toy_field_data)

    track_lf = _build(path_lf)
    track_crlf = _build(path_crlf)

    assert json_mod.dumps(track_lf.to_dict(), sort_keys=True) == json_mod.dumps(
        track_crlf.to_dict(), sort_keys=True
    )
