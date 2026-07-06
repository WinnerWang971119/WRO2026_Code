"""Tiny, self-closing localhost save-server for the offline layout editor.

Serves the already-rendered ``layout_editor.html`` over ``http://127.0.0.1`` so
the editor's Save can POST straight into ``tools/path_planner/layouts/layout.json``
— no download dialog, no browser download-folder change. This is the ``--editor``
front door's server leg (ADR-057).

**It cannot outlive the editing session.** The page heartbeats (``POST /ping``)
every few seconds while open and beacons ``POST /close`` on tab-close; a watchdog
thread self-terminates the server the moment heartbeats stop for
:data:`WATCHDOG_TIMEOUT_S` — a closed tab, a quit browser, a slept laptop, or a
crash all stop the beats, so no orphan server is ever left running. The socket
binds ``127.0.0.1`` only, so nothing off the machine can reach it.

Endpoints (all localhost, all no-cache):

* ``GET  /``      → the editor HTML.
* ``POST /save``  → write the posted JSON to the layout path (200 + path, or
  4xx/5xx with a message the editor surfaces).
* ``POST /ping``  → heartbeat (204); resets the watchdog.
* ``POST /close`` → graceful shutdown (204 then stop).

Stdlib only — no third-party dependencies, no persistent state.
"""

from __future__ import annotations

import json
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict

__all__ = ["serve_editor", "serve_page", "WATCHDOG_TIMEOUT_S"]

#: The editor heartbeats every ~3 s (assets/editor_io.js). If no request of any
#: kind arrives for this long, the browser is gone (tab closed / quit / asleep /
#: crashed) and the watchdog shuts the server down. Comfortably above the beat
#: interval so an ordinary edit never trips it, low enough that a closed tab
#: frees the port within a few seconds.
WATCHDOG_TIMEOUT_S: float = 12.0

#: How often the watchdog re-checks the idle clock.
_WATCHDOG_POLL_S: float = 1.0


class _EditorServer(ThreadingHTTPServer):
    """A localhost page server carrying the page bytes + an optional save target.

    ``layout_out`` is the destination a ``POST /save`` writes. It is optional:
    ``None`` runs the server in read-only "static page" mode (the player's
    ``--serve`` flow, which has nothing to save), and ``POST /save`` then 404s.
    """

    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, handler, html_bytes: bytes, layout_out=None):
        super().__init__(address, handler)
        self.html_bytes = html_bytes
        self.layout_out = layout_out
        self.last_activity = time.monotonic()
        self.stop_event = threading.Event()

    def touch(self) -> None:
        """Mark the connection alive so the watchdog holds off."""
        self.last_activity = time.monotonic()

    def request_shutdown(self) -> None:
        """Signal the watchdog to stop the server (from a /close or a timeout)."""
        self.stop_event.set()


class _EditorHandler(BaseHTTPRequestHandler):
    """Serve the editor page and accept its save / heartbeat / close posts."""

    # A closed browser tab drops the socket mid-response; the resulting
    # BrokenPipe/ConnectionReset is expected, not an error worth a traceback.
    def log_message(self, *_args) -> None:  # noqa: D401 - silence default logging
        pass

    # ----- helpers ---------------------------------------------------------

    def _send(self, code: int, ctype: str, body: bytes) -> None:
        try:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            if body:
                self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            # The tab went away between request and response — nothing to do.
            pass

    def _send_json(self, code: int, obj: Dict[str, Any]) -> None:
        self._send(code, "application/json", json.dumps(obj).encode("utf-8"))

    def _read_body(self) -> bytes:
        length = int(self.headers.get("Content-Length", 0) or 0)
        return self.rfile.read(length) if length > 0 else b""

    # ----- routes ----------------------------------------------------------

    def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        self.server.touch()
        if self.path in ("/", "/index.html", "/layout_editor.html"):
            self._send(200, "text/html; charset=utf-8", self.server.html_bytes)
        else:
            self._send(404, "text/plain; charset=utf-8", b"not found")

    def do_POST(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        self.server.touch()
        if self.path == "/save":
            self._handle_save()
        elif self.path == "/ping":
            self._send(204, "text/plain; charset=utf-8", b"")
        elif self.path == "/close":
            self._send(204, "text/plain; charset=utf-8", b"")
            self.server.request_shutdown()
        else:
            self._send(404, "text/plain; charset=utf-8", b"not found")

    def _handle_save(self) -> None:
        if self.server.layout_out is None:
            # Static-page mode (player --serve): there is no save target.
            self._send_json(404, {"ok": False, "error": "save not supported"})
            return
        raw = self._read_body()
        try:
            data = json.loads(raw.decode("utf-8"))
        except (ValueError, UnicodeDecodeError) as exc:
            self._send_json(400, {"ok": False, "error": f"invalid JSON: {exc}"})
            return
        # A minimal shape guard only — the PLAYER validates strictly at embed
        # time (ctps/layout.py); the editor already writes a well-formed dict, so
        # this just refuses obvious garbage rather than half-writing the file.
        if not isinstance(data, dict) or not isinstance(data.get("objects"), list):
            self._send_json(400, {"ok": False, "error": 'missing "objects" list'})
            return
        out: Path = self.server.layout_out
        try:
            out.parent.mkdir(parents=True, exist_ok=True)
            out.write_text(
                json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8"
            )
        except OSError as exc:
            self._send_json(500, {"ok": False, "error": str(exc)})
            return
        print(f"ctps: saved layout -> {out}", file=sys.stderr)
        self._send_json(200, {"ok": True, "path": str(out)})


def _watchdog(server: _EditorServer) -> None:
    """Shut the server down on an explicit /close or on heartbeat silence.

    Polls the idle clock; when either the stop event is set (a /close beacon) or
    no request has arrived for :data:`WATCHDOG_TIMEOUT_S`, calls
    ``server.shutdown()``. That covers every way the browser can vanish — a clean
    tab close (beacon), and every un-clean one (quit / sleep / crash), where the
    heartbeat simply stops — so the process always exits on its own.
    """
    while not server.stop_event.is_set():
        if time.monotonic() - server.last_activity > WATCHDOG_TIMEOUT_S:
            break
        server.stop_event.wait(_WATCHDOG_POLL_S)
    server.shutdown()


def _run_server(server: _EditorServer, url: str, note: str, open_browser: bool) -> int:
    """Start the watchdog, optionally open ``url``, then block in ``serve_forever``.

    The shared serving leg behind :func:`serve_editor` and :func:`serve_page`:
    it starts the auto-close :func:`_watchdog`, prints ``note`` (the ready line
    a caller/tool watches for), optionally launches the browser from a side
    thread (so a slow launch never delays the bind→serve handoff), and blocks
    until the watchdog (a /close beacon or heartbeat silence) or Ctrl-C stops it.

    :param server: an already-bound :class:`_EditorServer`.
    :param url: the served URL, opened when ``open_browser`` is set.
    :param note: the stderr "ready" message printed once serving begins.
    :param open_browser: when ``True``, open ``url`` after binding.
    :return: ``0`` on a clean shutdown.
    """
    watchdog = threading.Thread(target=_watchdog, args=(server,), daemon=True)
    watchdog.start()

    print(note, file=sys.stderr)
    if open_browser:
        threading.Thread(target=webbrowser.open, args=(url,), daemon=True).start()

    try:
        server.serve_forever(poll_interval=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


def serve_editor(
    html_bytes: bytes,
    layout_out: "str | Path",
    open_browser: bool = True,
    host: str = "127.0.0.1",
    port: int = 0,
) -> int:
    """Serve ``html_bytes`` on a localhost port until the browser tab goes away.

    Binds ``host:port`` (default ``127.0.0.1:0`` — an ephemeral free port, never
    exposed off the machine), starts the auto-close :func:`_watchdog`, optionally
    opens the page, and blocks in ``serve_forever`` until the watchdog (a /close
    beacon or heartbeat silence) or Ctrl-C stops it. Pass a fixed ``port`` when an
    external viewer (e.g. VS Code's Simple Browser) must open a known URL;
    ``port=0`` keeps the collision-free ephemeral default two editors can share.

    :param html_bytes: the rendered ``layout_editor.html`` to serve at ``/``.
    :param layout_out: destination path a ``POST /save`` writes (the layout).
    :param open_browser: when ``True``, open the served URL after binding.
    :param host: bind address; keep the localhost default.
    :param port: bind port; ``0`` (default) picks a free ephemeral port.
    :return: ``0`` on a clean shutdown; ``1`` if the port could not be bound.
    """
    try:
        server = _EditorServer(
            (host, port), _EditorHandler, html_bytes, Path(layout_out)
        )
    except OSError as exc:
        print(
            f"ctps: could not bind {host}:{port} for the layout editor ({exc}).\n"
            "  Another server may be using that port — free it or pick another "
            "with --port.",
            file=sys.stderr,
        )
        return 1
    bound = server.server_address[1]
    url = f"http://{host}:{bound}/"

    note = (
        f"ctps: layout editor at {url}\n"
        f"  Save writes {Path(layout_out)} directly; the server auto-closes a "
        f"few seconds after you close the tab."
    )
    result = _run_server(server, url, note, open_browser)
    print("ctps: layout editor closed.", file=sys.stderr)
    return result


def serve_page(
    html_bytes: bytes,
    open_browser: bool = True,
    host: str = "127.0.0.1",
    port: int = 0,
    label: str = "page",
) -> int:
    """Serve a static ``html_bytes`` page on a localhost port, self-closing.

    The read-only sibling of :func:`serve_editor` for the player's ``--serve``
    flow: it serves the page at ``/`` with the same heartbeat/watchdog auto-close
    (ADR-057) but carries no save target, so ``POST /save`` 404s. The page must
    heartbeat (``POST /ping`` + a ``/close`` beacon) for the watchdog to notice
    the tab is gone; a page that never pings simply frees the port after
    :data:`WATCHDOG_TIMEOUT_S`, so the server can still never dangle.

    :param html_bytes: the rendered page to serve at ``/``.
    :param open_browser: when ``True``, open the served URL after binding.
    :param host: bind address; keep the localhost default.
    :param port: bind port; ``0`` (default) picks a free ephemeral port. Pass a
        fixed port when an external viewer must open a known URL.
    :param label: a short noun for the log lines (e.g. ``"player"``); the ready
        line ``ctps: <label> at http://...`` is what a launching tool watches for.
    :return: ``0`` on a clean shutdown; ``1`` if the port could not be bound.
    """
    try:
        server = _EditorServer((host, port), _EditorHandler, html_bytes, None)
    except OSError as exc:
        print(
            f"ctps: could not bind {host}:{port} for the {label} ({exc}).\n"
            "  Another server may be using that port — free it or pick another "
            "with --port.",
            file=sys.stderr,
        )
        return 1
    bound = server.server_address[1]
    url = f"http://{host}:{bound}/"

    note = (
        f"ctps: {label} at {url}\n"
        f"  The server auto-closes a few seconds after you close the tab."
    )
    result = _run_server(server, url, note, open_browser)
    print(f"ctps: {label} closed.", file=sys.stderr)
    return result
