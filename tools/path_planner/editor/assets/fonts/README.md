# Editor fonts — fallback-only policy

This directory is the drop point for any font files the live field editor
would *privately* load at runtime. **It ships no `.ttf` / `.otf` files.**

## Why no bundled fonts

We do not vendor font binaries into the repo: they bloat the tree, carry their
own licensing obligations, and are not needed for the editor to look correct.
Instead, `editor/theme.py` defines each typographic role as an **ordered
fallback chain** of family names that ends in a generic family every platform
resolves:

| Role            | Chain (most preferred → generic)                                  |
| --------------- | ----------------------------------------------------------------- |
| Display / serif | `Fraunces` → `Cambria` → `Georgia` → `serif`                      |
| Sans UI         | `IBM Plex Sans` → `Segoe UI` → `Helvetica` → `sans-serif`         |
| Mono (numeric)  | `IBM Plex Mono` → `Cascadia Mono` → `Consolas` → `monospace`      |

`theme.resolve_font(chain)` walks the chain and returns the first family Tk
actually reports via `tkinter.font.families()`, falling back to the generic
tail when none of the named families are installed (or when there is no Tk root
yet, e.g. headless). On a typical Windows install the second entry in each
chain (`Cambria`, `Segoe UI`, `Consolas` / `Cascadia Mono`) resolves, so the
editor looks right with **zero** bundled assets.

## If you *do* want a specific font

Drop the `.ttf` / `.otf` here. `theme.load_bundled_fonts()` will best-effort
private-load every font file in this folder for the current process (Windows
`AddFontResourceExW` with `FR_PRIVATE`), so its family name becomes resolvable
by the chains above — just make sure the file's family name matches the first
entry of the relevant chain.

This is strictly **best-effort**: font loading is wrapped in `try/except` and a
load failure, an unsupported platform, or a missing file is silently ignored.
A missing or unloadable font must **never** crash the editor — the chain's
generic tail always wins.
