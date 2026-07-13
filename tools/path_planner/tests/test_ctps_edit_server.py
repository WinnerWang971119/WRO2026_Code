"""Hermetic tests for the localhost layout-editor save-server (ADR-057).

Binds an ephemeral ``127.0.0.1`` port (never external), drives it over HTTP with
``urllib``, and asserts the load-bearing behaviours:

* ``GET /`` serves the editor HTML;
* a valid ``POST /save`` writes the layout file (creating ``layouts/``);
* a malformed ``POST /save`` is refused **without** writing;
* ``POST /close`` stops the server;
* the watchdog self-terminates the server when heartbeats go silent — the "can
  never dangle" guarantee.

Every server runs in a daemon thread that the test joins with a timeout, so the
suite is self-contained and time-bounded regardless of host.
"""

from __future__ import annotations

import json
import threading
import urllib.error
import urllib.request
from pathlib import Path

import pytest

from tools.path_planner.ctps import edit_server

_VALID_LAYOUT = {
    "schema_version": 1,
    "objects": [
        {
            "id": "o1",
            "name": "tile",
            "shape": "rect",
            "type": "tile",
            "color": "#57b36b",
            "x_cm": 10.0,
            "y_cm": 20.0,
            "heading_deg": 0.0,
            "w_cm": 4.0,
            "h_cm": 4.0,
        }
    ],
    "robot": {},
}


def _make_server(html: bytes, layout_out: Path) -> edit_server._EditorServer:
    """Bind an ephemeral localhost editor server (not yet serving)."""
    return edit_server._EditorServer(
        ("127.0.0.1", 0), edit_server._EditorHandler, html, layout_out
    )


def _serve(server: edit_server._EditorServer) -> threading.Thread:
    """Run ``serve_forever`` in a joined daemon thread; return the thread."""
    thread = threading.Thread(
        target=server.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True
    )
    thread.start()
    return thread


def _serve_closing(server: edit_server._EditorServer) -> threading.Thread:
    """Serve ``server`` in a daemon thread that closes its socket on exit.

    Mirrors production's :func:`edit_server._run_server` (``server_close`` in a
    ``finally``) so that when the server is shut down — by an eviction ``/close``
    or an explicit ``shutdown()`` — its listening socket is released and the port
    frees, which the fixed-port rebind depends on.
    """
    def _run() -> None:
        try:
            server.serve_forever(poll_interval=0.05)
        finally:
            server.server_close()

    thread = threading.Thread(target=_run, daemon=True)
    thread.start()
    return thread


def _wait_until_serving(port: int, timeout: float = 3.0) -> None:
    """Poll ``GET /`` until the server answers, or fail after ``timeout``."""
    import time as _time

    deadline = _time.monotonic() + timeout
    while _time.monotonic() < deadline:
        try:
            with urllib.request.urlopen(
                f"http://127.0.0.1:{port}/", timeout=1
            ) as resp:
                resp.read()
                return
        except urllib.error.URLError:
            _time.sleep(0.02)
    raise AssertionError(f"server never came up on {port}")


def _free_fixed_port() -> int:
    """Grab a port then release it, so a server can bind that exact number."""
    import socket

    probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    return port


def test_bind_server_reopens_occupied_fixed_port(tmp_path):
    """A fixed-port reopen ends up with a live server on that SAME port.

    Models the operator's reopen: an orphaned old editor server still holds the
    fixed port when the VS Code task reopens. On macOS/Linux the new bind cannot
    take an active listener, so :func:`edit_server._bind_server` evicts the
    incumbent (``POST /close``) and rebinds — meaning a successful rebind PROVES
    the old server was released. (On Windows ``SO_REUSEADDR`` co-binds, which is
    why the original bug never reproduced there.) Either way the reopened server
    owns the fixed port the Simple Browser task hardcodes.
    """
    fixed = _free_fixed_port()

    old = edit_server._EditorServer(
        ("127.0.0.1", fixed), edit_server._EditorHandler,
        b"<html>OLD</html>", tmp_path / "old.json",
    )
    old_thread = _serve_closing(old)
    old_watchdog = threading.Thread(
        target=edit_server._watchdog, args=(old,), daemon=True
    )
    old_watchdog.start()
    _wait_until_serving(fixed)

    # Reopen onto the occupied fixed port: must come back bound to the same port.
    new = edit_server._bind_server(
        ("127.0.0.1", fixed), b"<html>NEW</html>", tmp_path / "new.json", None
    )
    try:
        assert new.server_address[1] == fixed
    finally:
        new.server_close()
        old.shutdown()  # no-op if eviction already stopped it (macOS/Linux)
        old_thread.join(timeout=3)
        old_watchdog.join(timeout=3)


def test_wait_port_free_returns_immediately_when_nothing_listening():
    """An unused fixed port is reported free at once (no wait on the common path)."""
    free = _free_fixed_port()  # grabbed then released -> nothing listening
    assert edit_server._wait_port_free("127.0.0.1", free, timeout=1.0) is True


def test_wait_port_free_shortcircuits_ephemeral_port():
    """``port == 0`` is meaningless to wait on and returns immediately."""
    assert edit_server._wait_port_free("127.0.0.1", 0, timeout=1.0) is True


def test_wait_port_free_times_out_while_server_holds_port(tmp_path):
    """A still-listening server keeps the port busy until ``timeout`` elapses."""
    fixed = _free_fixed_port()
    server = edit_server._EditorServer(
        ("127.0.0.1", fixed), edit_server._EditorHandler,
        b"<html>BUSY</html>", tmp_path / "busy.json",
    )
    thread = _serve_closing(server)
    _wait_until_serving(fixed)
    try:
        # The server is up, so the port never frees within the (short) window.
        assert edit_server._wait_port_free("127.0.0.1", fixed, timeout=0.5) is False
    finally:
        server.shutdown()
        thread.join(timeout=3)


def test_wait_port_free_unblocks_after_close_frees_port(tmp_path):
    """After ``/close`` releases the socket, the wait flips to free.

    Models the ``--stop-player`` flow: evict the incumbent, then block until the
    port is truly free so the follow-on ``--serve`` binds a clean port instead of
    racing a dying server — the fix for the alternating ok/fail reopen.
    """
    fixed = _free_fixed_port()
    server = edit_server._EditorServer(
        ("127.0.0.1", fixed), edit_server._EditorHandler,
        b"<html>OLD</html>", tmp_path / "old.json",
    )
    thread = _serve_closing(server)
    watchdog = threading.Thread(
        target=edit_server._watchdog, args=(server,), daemon=True
    )
    watchdog.start()
    _wait_until_serving(fixed)

    edit_server._request_close("127.0.0.1", fixed)  # ack is instant; exit lags
    try:
        assert edit_server._wait_port_free("127.0.0.1", fixed, timeout=3.0) is True
    finally:
        server.shutdown()  # no-op if the /close already stopped it
        thread.join(timeout=3)
        watchdog.join(timeout=3)


def test_bind_server_reraises_on_ephemeral_bind_failure(monkeypatch):
    """An ephemeral (``port==0``) bind failure is never masked by eviction retries.

    ``port==0`` can't collide, so a raised ``OSError`` there is a real fault and
    must propagate immediately rather than spin the eviction retry loop.
    """
    class _Boom(edit_server._EditorServer):
        def __init__(self, *_a, **_k):  # bypass the real bind, force a failure
            raise OSError("simulated bind failure")

    monkeypatch.setattr(edit_server, "_EditorServer", _Boom)
    with pytest.raises(OSError, match="simulated bind failure"):
        edit_server._bind_server(("127.0.0.1", 0), b"x", None, None)


def _post(port: int, path: str, body: bytes):
    """POST ``body`` to the server; return ``(status, response_bytes)``."""
    req = urllib.request.Request(
        f"http://127.0.0.1:{port}{path}",
        data=body,
        method="POST",
        headers={"Content-Type": "application/json"},
    )
    with urllib.request.urlopen(req, timeout=2) as resp:
        return resp.status, resp.read()


def test_get_serves_editor_html(tmp_path):
    """``GET /`` returns the served editor bytes."""
    server = _make_server(b"<html>EDITOR</html>", tmp_path / "layout.json")
    port = server.server_address[1]
    thread = _serve(server)
    try:
        with urllib.request.urlopen(
            f"http://127.0.0.1:{port}/", timeout=2
        ) as resp:
            assert resp.status == 200
            assert b"EDITOR" in resp.read()
    finally:
        server.shutdown()
        thread.join(timeout=2)
        server.server_close()
    assert not thread.is_alive()


def test_save_writes_layout(tmp_path):
    """A valid ``POST /save`` writes the layout into a (created) ``layouts/`` dir."""
    out = tmp_path / "layouts" / "layout.json"
    server = _make_server(b"<html></html>", out)
    port = server.server_address[1]
    thread = _serve(server)
    try:
        status, body = _post(
            port, "/save", json.dumps(_VALID_LAYOUT).encode("utf-8")
        )
        assert status == 200
        assert json.loads(body)["ok"] is True
        assert out.is_file()
        written = json.loads(out.read_text(encoding="utf-8"))
        assert written["objects"][0]["id"] == "o1"
    finally:
        server.shutdown()
        thread.join(timeout=2)
        server.server_close()


def test_get_reflects_current_html_after_change(tmp_path):
    """``GET /`` serves the LIVE ``current_html`` callback, not the start-time bytes.

    The fix for the fixed-port reopen bug: the page embed is regenerated on every
    GET, so once a Save has changed the on-disk layout a reload/refresh — even one
    that lands on the SAME still-alive server — shows the new layout instead of
    the empty scene baked in at startup. Modelled here with a mutable holder the
    callback reads: the first GET sees the start value, and after it changes the
    next GET reflects it (no server restart).
    """
    holder = {"body": b"<html>LAYOUT=null</html>"}
    server = edit_server._EditorServer(
        ("127.0.0.1", 0),
        edit_server._EditorHandler,
        b"<html>STALE START BYTES</html>",   # the one-time embed must NOT win
        tmp_path / "layout.json",
        current_html=lambda: holder["body"],
    )
    port = server.server_address[1]
    thread = _serve(server)
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/", timeout=2) as resp:
            first = resp.read()
        assert first == holder["body"]
        assert b"STALE START BYTES" not in first

        # A Save changes the on-disk layout -> the callback now returns new bytes.
        holder["body"] = b"<html>LAYOUT={objects:[o1]}</html>"
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/", timeout=2) as resp:
            second = resp.read()
        assert second == holder["body"]
        assert b"o1" in second
    finally:
        server.shutdown()
        thread.join(timeout=2)
        server.server_close()


def test_get_falls_back_to_html_bytes_when_callback_raises(tmp_path):
    """A throwing ``current_html`` degrades to the start-time bytes, never a 500."""
    def _boom():
        raise RuntimeError("disk gone")

    server = edit_server._EditorServer(
        ("127.0.0.1", 0),
        edit_server._EditorHandler,
        b"<html>FALLBACK</html>",
        tmp_path / "layout.json",
        current_html=_boom,
    )
    port = server.server_address[1]
    thread = _serve(server)
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/", timeout=2) as resp:
            assert resp.status == 200
            assert b"FALLBACK" in resp.read()
    finally:
        server.shutdown()
        thread.join(timeout=2)
        server.server_close()


def test_save_rejects_bad_json_without_writing(tmp_path):
    """A malformed ``POST /save`` returns 400 and writes nothing."""
    out = tmp_path / "layout.json"
    server = _make_server(b"<html></html>", out)
    port = server.server_address[1]
    thread = _serve(server)
    try:
        code = None
        try:
            _post(port, "/save", b"not json at all")
        except urllib.error.HTTPError as exc:
            code = exc.code
            exc.close()  # release the error response socket (no ResourceWarning)
        assert code == 400
        assert not out.exists()
    finally:
        server.shutdown()
        thread.join(timeout=2)
        server.server_close()


def test_close_stops_server(tmp_path):
    """``POST /close`` (via the watchdog) shuts the server down."""
    server = _make_server(b"x", tmp_path / "layout.json")
    port = server.server_address[1]
    thread = _serve(server)
    watchdog = threading.Thread(
        target=edit_server._watchdog, args=(server,), daemon=True
    )
    watchdog.start()

    status, _ = _post(port, "/close", b"")
    assert status == 204

    thread.join(timeout=3)
    watchdog.join(timeout=3)
    server.server_close()
    assert not thread.is_alive()


def test_static_page_mode_serves_but_refuses_save(tmp_path):
    """A ``layout_out=None`` server (player ``--serve``) serves ``/`` but 404s /save.

    The static-page sibling of the editor: ``GET /`` returns the page bytes, but
    since there is no save target a ``POST /save`` is refused (404) and nothing
    is written — the read-only guarantee the player serve relies on.
    """
    server = edit_server._EditorServer(
        ("127.0.0.1", 0), edit_server._EditorHandler, b"<html>PLAYER</html>", None
    )
    port = server.server_address[1]
    thread = _serve(server)
    try:
        with urllib.request.urlopen(
            f"http://127.0.0.1:{port}/", timeout=2
        ) as resp:
            assert resp.status == 200
            assert b"PLAYER" in resp.read()

        code = None
        try:
            _post(port, "/save", json.dumps(_VALID_LAYOUT).encode("utf-8"))
        except urllib.error.HTTPError as exc:
            code = exc.code
            exc.close()  # release the error response socket (no ResourceWarning)
        assert code == 404
    finally:
        server.shutdown()
        thread.join(timeout=2)
        server.server_close()


def test_serve_page_binds_requested_fixed_port(tmp_path):
    """``serve_page`` binds the exact port asked for so an external viewer's URL matches.

    Runs the blocking ``serve_page`` in a daemon thread on a specific free port
    (grabbed then released so it is almost certainly still free), then proves the
    page is reachable at THAT port — the property the VS Code Simple Browser task
    depends on (it opens a hardcoded ``http://127.0.0.1:<port>/``).
    """
    import socket
    import threading as _threading

    # Grab a free port, then release it so serve_page can bind the same number.
    probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    probe.bind(("127.0.0.1", 0))
    fixed_port = probe.getsockname()[1]
    probe.close()

    thread = _threading.Thread(
        target=edit_server.serve_page,
        args=(b"<html>FIXED</html>",),
        kwargs={"open_browser": False, "port": fixed_port},
        daemon=True,
    )
    thread.start()
    try:
        # Poll briefly for the bind→serve handoff.
        last_exc = None
        for _ in range(50):
            try:
                with urllib.request.urlopen(
                    f"http://127.0.0.1:{fixed_port}/", timeout=2
                ) as resp:
                    assert resp.status == 200
                    assert b"FIXED" in resp.read()
                break
            except urllib.error.URLError as exc:  # not yet listening
                last_exc = exc
                import time

                time.sleep(0.02)
        else:  # pragma: no cover - only if the server never came up
            raise AssertionError(f"serve_page never bound {fixed_port}: {last_exc}")
    finally:
        # Stop the server via its /close beacon so the daemon thread unwinds.
        try:
            _post(fixed_port, "/close", b"")
        except Exception:  # noqa: BLE001 - best-effort teardown
            pass
        thread.join(timeout=3)


def test_watchdog_auto_closes_on_heartbeat_silence(monkeypatch, tmp_path):
    """With no heartbeat, the watchdog shuts the server down on its own.

    The "can never dangle" guarantee: a browser that vanishes without a clean
    ``/close`` (quit / sleep / crash) simply stops pinging, and the idle watchdog
    terminates the server. A short timeout keeps the test fast.
    """
    monkeypatch.setattr(edit_server, "WATCHDOG_TIMEOUT_S", 0.3)
    server = _make_server(b"x", tmp_path / "layout.json")
    thread = _serve(server)
    watchdog = threading.Thread(
        target=edit_server._watchdog, args=(server,), daemon=True
    )
    watchdog.start()

    # Never touch the server: the watchdog must close it unprompted.
    thread.join(timeout=3)
    watchdog.join(timeout=3)
    server.server_close()
    assert not thread.is_alive()
