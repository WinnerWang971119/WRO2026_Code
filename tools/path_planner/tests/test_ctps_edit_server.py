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
