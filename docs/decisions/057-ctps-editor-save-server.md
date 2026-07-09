# ADR-057: Optional self-closing localhost save-server for the offline layout editor

**Status:** Implemented
**Date decided:** 2026-07 (issue #57 follow-up)

## Rule

The `--editor` front door (`python -m tools.path_planner.ctps --editor`) serves
the rendered `layout_editor.html` from a tiny localhost HTTP server
(`ctps/edit_server.py`) instead of opening it as a `file://` page. The editor's
**Save** POSTs the layout JSON to `/save`, which writes it straight to
`tools/path_planner/layouts/layout.json` — no browser download dialog, no
download-folder change. The socket binds `127.0.0.1` only, and the server
**auto-closes** when the editor tab goes away: the page heartbeats (`POST /ping`)
while open and beacons `POST /close` on unload, and a watchdog thread shuts the
server down the moment heartbeats stop for `WATCHDOG_TIMEOUT_S` — so a closed
tab, a quit browser, a slept laptop, or a crash all free it within seconds and no
orphan process is ever left. Opened directly as a `file://` page the editor still
falls back to the Blob download (ADR-056), and a *failed* server Save also falls
back to the download — no edit is ever lost.

## Why

ADR-056 chose Blob download / FileReader for editor persistence so the player
stays a self-contained `file://` artifact. But that leaves the author manually
routing each downloaded `layout.json` into place — or changing Chrome's global
download folder, which pollutes every other download — before the player's
auto-embed (newest `layouts/layout*.json`) can pick it up. A dev-time localhost
server closes that gap: Save lands exactly where the auto-embed reads, with zero
dialog. It touches only the *editor* (an authoring tool), never the player, so
the player's self-containment guarantee is untouched. The self-closing watchdog
is the load-bearing safety property — a server that could outlive the tab would
be a lurking orphan; heartbeat-silence shutdown makes that impossible.

## How to apply

- `ctps/edit_server.py` owns the server: `serve_editor(html_bytes, layout_out)`,
  the `_EditorServer` / `_EditorHandler`, and the `_watchdog`. Localhost +
  ephemeral port; stdlib only; no persistent state.
- `assets/editor_io.js` gates on `location.protocol !== "file:"`: served → POST
  `/save` + heartbeat/close-beacon; `file:` → the ADR-056 Blob download. A
  server-save failure falls back to the download.
- The self-containment lint still applies to `editor_io.js` — use relative fetch
  paths (`/save`, `/ping`, `/close`) and a `location.protocol` check; never a
  literal `http://` (a banned substring).

## Trigger to revisit

- If the editor ever must be usable with no local Python process at all (pure
  `file://`), keep the download path as the fallback.
- If a second client (not the single editor tab) ever needs the server, revisit
  the localhost-only / single-session / heartbeat-owns-liveness assumptions.

## See also

- [ADR-056 — CTPS object/cargo model + the offline layout editor](./056-ctps-object-cargo.md) — the editor and its Blob-download persistence this server augments (and falls back to).
- `tools/path_planner/layouts/README.md` — the author-facing edit → save → view flow.
